"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — central cognitive coordinator of Grace.
Orchestrates the full perception-cognition-response cycle each conversational turn.
Milestone 1: Chatbot with Working Memory (WMC) + Episodic Memory (EMC)

Inference routed through Generative Cognitive Engine (GCE).
Model and endpoint configured via HRS.

Architecture:
    CNC mirrors the human thalamus — central relay between sensory input,
    memory systems, and motor output. It owns no I/O directly.
    All sensory input arrives from TIC via the sensory gateway topic.
    All cognitive output is published to the motor gateway topic for TOC to relay.
    All memory operations delegate to MCC — CNC never touches WMC or EMC directly.

    Each turn follows a fixed pipeline:
        1. Receive normalized SSS from TMS sensory gateway
        2. Register user turn in MCC (WMC fill + EMC eviction)
        3. Assemble unified memory context (WMC PMTs + EMC episodes)
        4. Build system prompt with date and reinstated episodic context
        5. Stream inference through GCE
        6. Register assistant turn in MCC
        7. Report memory stats

    Async architecture:
        ROS2 spin runs on main thread.
        A dedicated asyncio loop runs on a background thread (cnc-cognitive-cycle).
        All async operations (memory, inference) are scheduled onto this loop
        via run_coroutine_threadsafe — ROS2 callbacks never block.

Topics:
    Sub: TMS.TEXT_SENSORY_GATEWAY   (std_msgs/String) — normalized SSS from TIC
    Pub: TMS.TEXT_MOTOR_GATEWAY     (std_msgs/String) — CRS fragments to TOC
    Pub: SCS.MEMORY_CONTEXT_GATEWAY (std_msgs/String) — full GCE input context for debug
    Pub: SCS.MEMORY_STATS_GATEWAY   (std_msgs/String) — memory cortex stats after every turn

Response format (JSON on TMS.TEXT_MOTOR_GATEWAY):
    {"type": "start", "content": "<first fragment>"}
    {"type": "delta", "content": "<fragment>"}
    {"type": "done",  "content": "<full cognitive response>"}
    {"type": "error", "content": "<error message>"}

Terminology:
    Gamma Rhythm    — asyncio loop coordinating active cognition across memory and inference
    Neural Gateway  — persistent async HTTP connection to an external cognitive engine
    Neural Thread   — dedicated background thread hosting an async event loop
    Thalamic Relay  — role of the CNC in routing signals between sensory input, memory systems, and motor output
    CRS             — Cognitive Response Signal — Grace's generated output, efference copy for memory encoding
"""

# System components
import asyncio                                             # dedicated event loop for async memory and inference operations — runs on cnc-cognitive-cycle thread
from concurrent.futures import ThreadPoolExecutor          # thread pool for offloading blocking operations from the ROS2 spin thread
from datetime import datetime                              # for injecting current date into system prompt each turn
import httpx                                               # async HTTP client for GCE streaming — OpenAI-compatible SSE
import json                                                # for serializing ROS2 message payloads and parsing GCE SSE chunks
import threading                                           # for cnc-cognitive-cycle background thread hosting the event loop

# ROS2 components
import rclpy                                               # ROS2 Python client library — node lifecycle and spin
from rclpy.node import Node                                # base class for all ROS2 nodes
from rclpy.executors import MultiThreadedExecutor          # allows concurrent callback execution — required for async scheduling
from std_msgs.msg import String                            # ROS2 string message type for text I/O

# AGi components
from hrs.hrm import AGi                                    # homeostatic regulation manifest namespace
from hrs.hru import hydrate_manifest, UserAccessLevel      # manifest hydration + user type enum — binds AuRoRA parameter server values into AGi constants at node init
from scs.mcc import MemoryCoordinationCore                 # memory coordinator — CNC never touches WMC or EMC directly
from gms.csb import SensoryInputChannel, TraceType, SSS, CRS  # type: ignore[import-untyped] — SSS/CRS dataclasses + channel/trace enums
from scs.iru import IdentityRecognitionUnit                # identity recognition unit — identify user and establish session identity and user context
from scs.ppu import PersonalProgressionUnit                # Personal Provisioning Unit — session identity and user context loader

TMS = AGi.TMS                                              # module-level alias — TMS-level constants (topic names, websocket config)
SCS = AGi.SCS                                              # module-level alias — SCS-level constants (topic names, cortical capacity)
GCE = AGi.SCS.GCE                                          # module-level alias — GCE constants (model, endpoint, inference parameters)


class CNC(Node):
    """
    Central Neural Core — ROS2 node.

    Subscribes to normalized SSS from TIC, routes inference through GCE with streaming,
    publishes CRS fragments to motor gateway for TOC relay.
    Memory is managed entirely through MCC.
    CNC owns no I/O — it only thinks.
    """

    def __init__(self):
        """
        Initialize Central Neural Core — the main orchestrator of the robot's cognitive
        architecture.

        Initializes all cognitive subsystems in order:
            - Reticular Activating Compartment (RAC) — activation reaction that hydrates AGi and spawns all nodes
            - Generative Cognitive Engine (GCE) — persistent connection for inference of generative cognition
            - Memory Coordination Cortex (MCC) — short-term and long-term memory management
            - Neural threads — dedicated asyncio loops for parallel operations
            - Neural gateways — ROS2 topics for sensory input and motor output
        """
        super().__init__("cnc")                                             # register this node with ROS2 as "cnc"
        self.get_logger().info("=" * 60)
        self.get_logger().info("🧠 CNC — Central Neural Core starting…")
        self.get_logger().info("=" * 60)

        self._gce_inference_packet: dict = {                                # setup inference parameters of GCE
            "model"              : GCE.COGNITIVE_ENGINE,                    # GCE model identifier from HRS
            "messages"           : "",                                      # placeholder for input prompt
            "num_ctx"            : GCE.CONTEXT_WINDOW,                      # override Ollama default 2048 — allocate full model context
            "max_tokens"         : GCE.RESPONSE_DEPTH,                      # maximum response tokens per inference
            "temperature"        : GCE.TEMPERATURE,                         # response creativity
            "top_p"              : GCE.PROBABILITY_THRESHOLD,               # cumulative probability cutoff
            "top_k"              : GCE.CANDIDATE_THRESHOLD,                 # maximum candidate tokens per step
            "repetition_penalty" : GCE.PERSEVERATION_DAMPING,               # suppresses repetition
            "frequency_penalty"  : GCE.HABITUATION_DAMPING,                 # suppresses frequent tokens
            "presence_penalty"   : GCE.NOVELTY_BIAS,                        # bias toward new topics
            "stream"             : True,                                    # enable SSE streaming — fragments published as they arrive
        }

        # Initialize configuration through hydration
        hydrate_manifest(self, system="scs")                                # hydrate manifest from AuRoRA parameters under the SCS system
        self._active_user: str = AGi.ACTIVE_USER                            # (TODO): M1 stub — replace with login sequence
        self._iru = IdentityRecognitionUnit(logger=self.get_logger())       # boot identity recognition — loads user profile
        self._iru.recognize_user(user_id=self._active_user)                 # recognize active user — loads relational context
        self._access_level: UserAccessLevel = self._iru.user_profile.access_level  # access classification — governs recall scope

        self._ppu = PersonalProgressionUnit(logger=self.get_logger())       # boot personal progression — loads Grace's persona
        self._ppu.provision_cognition(user_profile=self._iru.user_profile)  # assemble system prompt — persona + user context

        # Initialize separate execution thread for memory and blocking operations
        self._cognitive_executor: ThreadPoolExecutor = ThreadPoolExecutor(  # thread pool for blocking operations — offloads from cognitive cycle
            max_workers=2,                                                  # two workers — memory and inference can run concurrently
            thread_name_prefix="cnc-thread-pool",                           # named for thread dump debugging
        )

        # Initialize GCE gateway to interface with generative cognitive engine
        self._gce_gateway: httpx.AsyncClient = httpx.AsyncClient(           # persistent async HTTP client — reused across all GCE requests
            base_url=GCE.NEURAL_ENDPOINT,                                   # base URL set once — requests only need the endpoint path
            timeout=httpx.Timeout(GCE.TIMEOUT),                             # applies timeout to connect, read, and write operations
        )

        # Initialize MCC for memory management
        self.mcc: MemoryCoordinationCore = MemoryCoordinationCore(          # boot memory coordination — WMC + EMC, embedding model loads here
            logger=self.get_logger(),
            executor=self._cognitive_executor,
        )

        # Initialize neural thread for parallel operations
        self._cognitive_cycle: asyncio.AbstractEventLoop = asyncio.new_event_loop()  # asyncio event loop for active cognition — isolated from ROS2 spin thread
        self._gamma_rhythm: threading.Thread = threading.Thread(            # dedicated OS thread driving the cognitive cycle
            target=self._cognitive_cycle.run_forever,
            name="cnc-cognitive-cycle",
            daemon=True,                                                    # dies with main process — clean shutdown
        )
        self._gamma_rhythm.start()                                          # ignite gamma rhythm — cognitive cycle now active

        # Sensory gateway — normalized SSS arrives here from TIC
        self._sensory_input: rclpy.subscription.Subscription = self.create_subscription(
            String, TMS.TEXT_SENSORY_GATEWAY, self._receive_stimulus, 10   # String type | topic | callback | QoS depth 10
        )

        # Motor gateway — CRS fragments published here for TOC to relay to WebUI
        self._motor_output: rclpy.publisher.Publisher = self.create_publisher(
            String, TMS.TEXT_MOTOR_GATEWAY, 10                             # String type | topic | QoS depth 10
        )

        # Debug and monitoring gateways
        self._memory_context_feedback: rclpy.publisher.Publisher = self.create_publisher(
            String, SCS.MEMORY_CONTEXT_GATEWAY, 10                         # full GCE input context — debug topic
        )
        self._memory_stats_feedback: rclpy.publisher.Publisher = self.create_publisher(
            String, SCS.MEMORY_STATS_GATEWAY, 10                           # memory cortex stats — monitoring topic
        )

        # Initialize attentional gate — True while processing a turn, queues incoming stimuli
        self._attention_gate: bool = False                                  # attentional gate — True while processing, False when ready
        self._pending_stimulus: SSS | None = None                          # single-slot busy queue — holds one SSS while CNC is processing

        asyncio.run_coroutine_threadsafe(                                   # submit GCE priming to cognitive cycle — fire and forget, doesn't block init
            self._prime_gce(), self._cognitive_cycle
        )

        self.get_logger().info(f"✅ Endpoint    : {GCE.NEURAL_ENDPOINT}")
        self.get_logger().info(f"✅ Model       : {self._gce_inference_packet['model']}")
        self.get_logger().info(f"✅ Subscribed  : {TMS.TEXT_SENSORY_GATEWAY}")
        self.get_logger().info(f"✅ Publishing  : {TMS.TEXT_MOTOR_GATEWAY}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🌸 GRACE is ready")
        self.get_logger().info("=" * 60)

    def _receive_stimulus(self, msg: String) -> None:
        """
        Receive a normalized SSS from the TMS sensory gateway and schedule cognitive processing.
        Queues one SSS if cognitive cycle is already processing a previous stimulus.
        Single-slot, latest-wins semantics: newest stimulus overwrites pending if queue is occupied.

        Args:
            msg (String): ROS2 string message carrying serialized SSS JSON from TIC.
        """
        try:
            payload: dict = json.loads(msg.data.strip())                                        # deserialize SSS JSON — TIC guarantees valid structure
            if not isinstance(payload, dict) or not payload.get("text"):                        # malformed or empty — reject at boundary
                return

            sss: SSS = SSS(                                                                     # reconstruct typed SSS from serialized fields
                role       = payload.get("role", "user"),
                text       = payload["text"].strip(),
                user_id    = payload.get("user_id", "demo"),
                source     = SensoryInputChannel(payload.get("source", SensoryInputChannel.WEBUI.value)),
                trace_type = TraceType(payload.get("trace_type", TraceType.PMT.value)),
            )
        except (json.JSONDecodeError, ValueError):
            return                                                                               # malformed SSS — drop at boundary, TIC should not send this

        if self._attention_gate:                                                                 # cognitive cycle busy — queue or overwrite
            if self._pending_stimulus is not None:
                self.get_logger().warning("⚠️  Cognitive Engine queue overwritten — previous stimulus dropped")
            self._pending_stimulus = sss                                                        # overwrite with latest — most recent intent takes priority
            self.get_logger().info("⏳ Cognitive Engine busy — stimulus queued")
            return

        self._attention_gate = True                                                             # close gate before scheduling — prevents TOCTOU
        asyncio.run_coroutine_threadsafe(
            self._process_stimulus(sss), self._cognitive_cycle                                  # submit to gamma rhythm — never blocks ROS2 spin
        )

    async def _process_stimulus(self, stimulus: SSS) -> None:
        """
        Process one full stimulus-response cycle.

        Registers the user SSS in memory, assembles the full memory context
        (WMC PMTs + recalled EMC episodes), builds the system prompt with episodic
        context injected, streams inference through GCE, registers the CRS in memory,
        then reports memory stats.

        Args:
            stimulus (SSS): Normalized sensory stimulus from TIC.
        """
        cognitive_response: str = ""                                                            # accumulates GCE response chunks — empty until inference completes

        try:
            # 1. Register user turn in memory
            await self.mcc.register_interaction_stimulus(stimulus)                              # park SSS in open episode slot — awaiting CRS pairing

            # 2. Assemble memory context
            memory_context: list[dict] = await self.mcc.assemble_memory_context(
                user_id    = self._active_user if self._access_level == UserAccessLevel.GUEST else None,
                user_prompt = stimulus.text,
            )

            # 3. Separate system and conversation parts
            long_term_memory: list[dict]  = [m for m in memory_context if m["role"] == "system"]   # episodic context blocks
            short_term_memory: list[dict] = [m for m in memory_context if m["role"] != "system"]   # WMC conversation turns

            # 4. Assemble system prompt — persona + date + episodic context
            active_cognition: str = self._ppu.active_cognition.format(
                date=datetime.now().strftime("%Y-%m-%d")                                        # inject current date in ISO-8601 format
            )

            if long_term_memory:                                                                # episodic context available — fuse with persona
                system_content: str = active_cognition + "\n\n" + "\n\n".join(m["content"] for m in long_term_memory)
            else:
                system_content: str = active_cognition                                          # no episodic context — persona prompt only

            # 5. Build final message list
            messages: list[dict] = [{"role": "system", "content": system_content}]             # system prompt — always first
            messages.extend(short_term_memory)                                                  # WMC PMTs — chronological conversation history
            messages.append({"role": "user", "content": stimulus.text})                         # current stimulus — last message before inference

            context_signal = String()
            context_signal.data = json.dumps({"messages": messages})
            self._memory_context_feedback.publish(context_signal)                               # publish full context — debug topic

            # 6. Stream from GCE
            self.get_logger().debug(f"📤 GCE messages: {messages}")
            cognitive_response = await self._stream_gce(messages)                               # stream GCE response — CRS fragments published to motor gateway as they arrive

            # 7. Register interaction — pair SSS + CRS into complete episode for memory
            if cognitive_response:
                crs = CRS(                                                                      # CRS — Grace's cognitive output, not a sensory signal
                    text       = cognitive_response,
                    trace_type = TraceType.PMT,
                )
                await self.mcc.register_interaction_response(crs)                              # complete episode: pair open SSS + CRS → PMT → WMC

            # 8. Report memory stats
            stats = self.mcc.report_memory_stats()
            stats_signal = String()
            stats_signal.data = json.dumps(stats)
            self._memory_stats_feedback.publish(stats_signal)

        except Exception as e:
            self.get_logger().error(f"❌ CNC handle error: {e}")
            self._emit_crs({"type": GCE.STREAM_ANOMALY, "content": str(e)})

        finally:
            self._attention_gate = False                                                        # reopen attentional gate — ready for next stimulus
            if self._pending_stimulus:                                                          # queued stimulus present — drain immediately
                pending = self._pending_stimulus
                self._pending_stimulus = None                                                   # clear slot before scheduling — gate reopens clean
                self._attention_gate = True                                                     # close gate before scheduling — queued stimulus now owns the cycle
                asyncio.ensure_future(self._process_stimulus(pending))                          # schedule queued stimulus on the running cognitive cycle

    async def _stream_gce(self, messages: list[dict]) -> str:
        """
        Generate and emit a cognitive response through the Generative Cognitive Engine.
        Response is streamed as CRS fragments to the motor gateway as it arrives.

        Args:
            messages (list[dict]): Full memory context and current user prompt for inference.

        Returns:
            str: Full cognitive response for memory registration as CRS.
        """
        cognitive_response: str = ""
        leading_fragment: bool = True
        self._gce_inference_packet["messages"] = messages

        try:
            async with self._gce_gateway.stream(
                "POST",
                "/v1/chat/completions",
                json=self._gce_inference_packet,
            ) as response:

                if response.status_code != 200:
                    anomaly: str = f"GCE HTTP {response.status_code}"
                    self.get_logger().error(f"❌ {anomaly}")
                    self._emit_crs({"type": GCE.STREAM_ANOMALY, "content": anomaly})
                    return ""

                async for line in response.aiter_lines():
                    if not line or not line.startswith("data:"):
                        continue

                    fragment_data: str = line[len("data:"):].strip()
                    if fragment_data == "[DONE]":
                        break

                    try:
                        fragment: dict = json.loads(fragment_data)
                    except json.JSONDecodeError:
                        continue

                    fragment_content: str = (
                        fragment.get("choices", [{}])[0]
                        .get(GCE.STREAM_PROPAGATING, {})
                        .get("content", "")
                    )
                    if not fragment_content:
                        continue

                    cognitive_response += fragment_content

                    if leading_fragment:
                        self._emit_crs({"type": GCE.STREAM_LEADING, "content": fragment_content})  # first fragment — start event
                        leading_fragment = False
                    else:
                        self._emit_crs({"type": GCE.STREAM_PROPAGATING, "content": fragment_content})  # subsequent fragments — delta event

            if cognitive_response:
                self._emit_crs({"type": GCE.STREAM_TRAILING, "content": cognitive_response})   # stream complete — done event
                self.get_logger().info(f"✅ Response: {len(cognitive_response)} chars")
            else:
                self._emit_crs({"type": GCE.STREAM_ANOMALY, "content": "Empty response from GCE"})

        except httpx.TimeoutException:
            err = "GCE timeout — model may still be loading"
            self.get_logger().error(f"❌ {err}")
            self._emit_crs({"type": GCE.STREAM_ANOMALY, "content": err})

        except Exception as e:
            self.get_logger().error(f"❌ Stream error: {e}")
            self._emit_crs({"type": GCE.STREAM_ANOMALY, "content": str(e)})

        return cognitive_response

    async def _prime_gce(self) -> None:
        """
        Prime GCE — activate the cognitive engine and pin it for the session.
        Fire and forget — called once at boot, non-blocking.
        """
        try:
            inference_packet: dict = {
                "model"     : GCE.COGNITIVE_ENGINE,
                "messages"  : [{"role": "user", "content": "hi"}],
                "max_tokens": 1,
                "stream"    : False,
            }
            await self._gce_gateway.post("/v1/chat/completions", json=inference_packet)
            self.get_logger().info("✅ GCE primed successfully — activated into memory")
        except Exception as e:
            self.get_logger().warning(f"⚠️ GCE priming failed: {e}")              # non-fatal — model loads on first real request

    def _emit_crs(self, response_packet: dict) -> None:
        """
        Emit a CRS fragment to the motor gateway topic for TOC relay.

        Args:
            response_packet (dict): CRS payload — type and content fields.
        """
        try:
            signal = String()
            signal.data = json.dumps(response_packet)
            self._motor_output.publish(signal)                                  # publish to motor gateway — TOC picks up and relays to WebUI
        except Exception as e:
            self.get_logger().error(f"❌ CRS emit error: {e}")                  # non-fatal — publish failure must not crash the cognitive cycle

    def destroy_node(self) -> None:
        """
        Gracefully shut down CNC — close all cognitive subsystems in reverse boot order.
        """
        self.get_logger().info("🛑 CNC shutting down…")
        self.mcc.close()

        future = asyncio.run_coroutine_threadsafe(
            self._gce_gateway.aclose(), self._cognitive_cycle
        )
        try:
            future.result(timeout=3.0)
        except Exception:
            pass

        self._cognitive_cycle.call_soon_threadsafe(self._cognitive_cycle.stop)
        self._gamma_rhythm.join(timeout=3.0)
        self._cognitive_executor.shutdown(wait=True)
        super().destroy_node()
        self.get_logger().info("✅ CNC shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = CNC()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutdown requested")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
