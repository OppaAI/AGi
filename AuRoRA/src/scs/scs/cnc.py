"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — central cognitive coordinator of Grace.
Orchestrates the full perception-cognition-response cycle each conversational turn.
Milestone 1: Chatbot with Working Memory (WMC) + Episodic Memory (EMC)

Inference routed through Generative Cognitive Engine (GCE).
Model and endpoint configured via HRP.

Architecture:
    CNC mirrors the human thalamus — the central relay that routes signals
    between sensory input, memory systems, and motor output.
    It owns no memory directly; all memory operations delegate to MCC.

    Each turn follows a fixed pipeline:
        1. Receive text input signal via ROS2 subscription
        2. Register user turn in MCC (WMC fill + EMC eviction)
        3. Assemble unified memory context (WMC PMTs + EMC episodes)
        4. Build system prompt with date and reinstated episodic context
        5. Stream inference through GCE
        6. Register assistant turn in MCC
        7. Report memory stats

    Async architecture:
        ROS2 spin runs on main thread.
        A dedicated asyncio loop runs on a background thread (cnc-asyncio).
        All async operations (memory, inference) are scheduled onto this loop
        via run_coroutine_threadsafe — ROS2 callbacks never block.

Topics:
    Sub: CNS.TEXT_INPUT_GATEWAY (std_msgs/String) — incoming text input signal
    Pub: GCE.RESPONSE_GATEWAY   (std_msgs/String) — streamed cognitive response

Response format (JSON on GCE.RESPONSE_GATEWAY):
    {"type": "start",  "content": "<first chunk>"}
    {"type": "delta",  "content": "<delta>"}
    {"type": "done",   "content": "<full response>"}
    {"type": "error",  "content": "<error message>"}

Terminology:
    Gamma Rhythm    — asyncio loop coordinating active cognition across memory and inference
    Neural Gateway  — persistent async HTTP connection to an external cognitive engine
    Neural Thread   — dedicated background thread hosting an async event loop
    Thalamic Relay  — role of the CNC in routing signals between sensory input, memory systems, and motor output.

TODO:
    M1.x — define and lock NeuralTextInput schema in scs/types.py
           standardize JSON contract across all input sources (CLI, web, voice)
           evaluate custom ROS2 msg type vs JSON bridge for non-ROS interfaces
    M2 — _strip_model_artifacts(): strip think blocks and roleplay artifacts
         from assistant response before handing to MCC for memory storage
    M2 — salience gate: score assistant response before registering in MCC —
         discard low-salience turns at the CNC boundary
    M2 — busy queue: buffer incoming inputs during processing rather than dropping
    M3 — emotional state initialization
    M3 — multi-modal input: image and audio signal routing through CNC
    M? — perception subsystems (visual, auditory)
    M? — motor control interface
"""

# System libraries
import asyncio                                             # dedicated event loop for async memory and inference operations — runs on cnc-asyncio thread
from concurrent.futures import ThreadPoolExecutor          # thread pool for offloading blocking operations from the ROS2 spin thread
from datetime import datetime                              # for injecting current date into system prompt each turn
import httpx                                               # async HTTP client for GCE streaming — OpenAI-compatible SSE
import json                                                # for serializing ROS2 message payloads and parsing GCE SSE chunks
import threading                                           # for cnc-asyncio background thread hosting the event loop

# ROS2 libraries
import rclpy                                               # ROS2 Python client library — node lifecycle and spin
from rclpy.node import Node                                # base class for all ROS2 nodes
from rclpy.executors import MultiThreadedExecutor          # allows concurrent callback execution — required for async scheduling
from std_msgs.msg import String                            # ROS2 string message type for text I/O

# AGi libraries
from scs.mcc import MemoryCoordinationCore                 # memory coordinator — CNC never touches WMC or EMC directly
from hrs.hrp import AGi                                    # homeostatic regulation parameter namespace
CNS = AGi.CNS                                              # CNS-level constants — topic names, cortical capacity
GCE = AGi.CNS.GCE                                          # GCE constants — model, endpoint, inference parameters


class CNC(Node):
    """
    Central Neural Core — ROS2 node.

    Subscribes to text input, routes inference through GCE with streaming,
    publishes response chunks to the cognitive response topic.
    Memory is managed entirely through MCC.
    """

    def __init__(self):
        """
        Initialize Central Neural Core — the main orchestrator of the robot's cognitive
        architecture.

        Initializes all cognitive subsystems in order:
            - Generative Cognitive Engine (GCE) — persistent connection for inference of generative cognition
            - Memory Coordination Core (MCC) — short-term and long-term memory management
            - Neural threads — dedicated asyncio loops for parallel operations
            - Neural gateways — ROS2 topics for text input and cognitive output


        """
        super().__init__("cnc")                                             # register this node with ROS2 as "cnc"
        self.get_logger().info("=" * 60)                                    # visual separator — stdout and /rosout
        self.get_logger().info("🧠 CNC — Central Neural Core starting…")    # boot announcement — stdout and /rosout
        self.get_logger().info("=" * 60)                                    # visual separator — stdout and /rosout

        
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
            logger=self.get_logger(),                                       # logger for memory coordination operations
            executor=self._cognitive_executor,                              # thread pool for memory coordination operations
        )

        # Initialize neural thread for parallel operations
        self._cognitive_cycle: asyncio.AbstractEventLoop = asyncio.new_event_loop()  # asyncio event loop for active cognition — isolated from ROS2 spin thread
        self._gamma_rhythm: threading.Thread = threading.Thread(            # dedicated OS thread driving the cognitive cycle
            target=self._cognitive_cycle.run_forever,                       # runs indefinitely — processes coroutines as they arrive
            name="cnc-cognitive-cycle",                                     # named for thread dump debugging
            daemon=True,                                                    # dies with main process — clean shutdown
        )
        self._gamma_rhythm.start()                                          # ignite gamma rhythm — cognitive cycle now active

        # Initialize neural gateway for text input and generative cognitive output
        self._text_input_stimulus: rclpy.subscription.Subscription = self.create_subscription(  # register ROS2 subscriber — fires on every incoming message
            String, CNS.TEXT_INPUT_GATEWAY, self._receive_text_input, 10    # String type | topic | callback | QoS depth 10
        )
        self._cognitive_response: rclpy.publisher.Publisher = self.create_publisher( # ROS2 publisher — sends cognitive output to the specified topic
            String, GCE.RESPONSE_GATEWAY, 10                                # String type | topic | QoS depth 10
        )

        # Initialize attentional gate — True while processing a turn, drops incoming stimuli
        self._attention_gate: bool = False                                  # attentional gate — True while processing a turn, drops incoming stimuli
        asyncio.run_coroutine_threadsafe(                                   # submit GCE priming to cognitive cycle — fire and forget, doesn't block init
            self._prime_gce(), self._cognitive_cycle                        # schedules across thread boundary — safe from ROS2 main thread
        )

        self.get_logger().info(f"✅ Endpoint    : {GCE.NEURAL_ENDPOINT}")   # confirm GCE endpoint
        self.get_logger().info(f"✅ Model       : {GCE.COGNITIVE_ENGINE}")  # confirm GCE model
        self.get_logger().info(f"✅ Subscribed  : {CNS.TEXT_INPUT_GATEWAY}")# confirm input topic
        self.get_logger().info(f"✅ Publishing  : {GCE.RESPONSE_GATEWAY}")  # confirm output topic
        self.get_logger().info("=" * 60)                                    # visual separator
        self.get_logger().info("🌸 GRACE is ready")                         # boot complete
        self.get_logger().info("=" * 60)                                    # visual separator

    def _receive_text_input(self, msg: String):
        """
        Receive an incoming text input signal and schedule cognitive processing.
        Drops the signal if CNC is already processing a turn.
        """
        try:                                                                        # attempt to parse JSON payload — CLI sends {"text": "..."}
            ui_input: dict = json.loads(msg.data.strip())                           # converts the ROS2 String message to a Python dictionary
            if not isinstance(ui_input, dict) or not ui_input.get("text"):          # malformed or empty payload — discard
                return                                                              # abort early
            user_prompt: str = ui_input.get("text", "").strip()                     # extract text field — strip whitespace
        except json.JSONDecodeError:                                                # plain string — not JSON, use as-is
            user_prompt = msg.data.strip()                                          # strip whitespace
        
        if not user_prompt:                                                         # empty input after stripping — discard
            return                                                                  # abort early

        if self._attention_gate:                                                    # cognitive cycle busy — drop incoming stimulus
            self.get_logger().warning("⚠️  Cognitive Engine is busy — dropping input") # log the cognitive cycle congestion
            self._emit_response({"type": "error", "content": "Cognitive Engine is still thinking…"}) # publish error notification
            return                                                                  # abort early

        user_prompt_chunk: int = len(user_prompt) // CNS.UNITS_PER_CHUNK + 1        # estimate chunk cost of incoming stimulus
        self.get_logger().info(f"📝 stimulus: {user_prompt_chunk} chunks")          # log chunk cost of user prompt

        asyncio.run_coroutine_threadsafe(                                           # schedule cognitive pipeline — crosses thread boundary safely
            self._process_stimulus(user_prompt), self._cognitive_cycle              # submit to gamma rhythm — never blocks ROS2 spin
        )

    async def _process_stimulus(self, user_prompt: str):
        """
        Process one full stimulus-response cycle:
            1. Register user prompt in memory
            2. Assemble memory context (WMC + EMC)
            3. Build system prompt with episodic context
            4. Stream inference through GCE
            5. Register AI response in memory
            6. Report memory stats
        """
        self._attention_gate = True                                             # close attentional gate — drop incoming stimuli during processing
        cognitive_response: str = ""                                            # accumulates GCE response chunks — empty until inference completes

        try:                                                                    # wrap full pipeline — any failure publishes error and resets attention gate
            # 1. Register user turn in memory
            await self.mcc.register_memory("user", user_prompt)                 # induce user turn into WMC — evicted PMTs bound to EMC asynchronously

            # 2. Assemble memory context
            memory_context: list[dict] = await self.mcc.assemble_memory_context(user_prompt)  # assemble full memory context — EMC episodes + WMC PMTs

            # 3. Separate system and conversation parts
            long_term_memory: list[dict] = [m for m in memory_context if m["role"] == "system"]    # extract system blocks
            short_term_memory: list[dict]  = [m for m in memory_context if m["role"] != "system"]  # extract conversation turns

            # 4. Assemble system prompt with date
            system_prompt: str = GCE.SYSTEM_PROMPT.format(                      # inject current date into system prompt
                date=datetime.now().strftime("%Y-%m-%d")                        # ISO date — Grace knows what day it is
            )
            if long_term_memory:                                                # episodic context available — append to system prompt
                system_content: str = system_prompt + "\n\n" + "\n\n".join(m["content"] for m in long_term_memory)  # fuse personality + episodic context
            else:
                system_content: str = system_prompt                             # no episodic context — personality prompt only

            # 5. Build final message list
            messages: list[dict] = [{"role": "system", "content": system_content}]  # system prompt — always first
            messages.extend(short_term_memory)                                  # inject WMC PMTs — chronological conversation history
            messages.append({"role": "user", "content": user_prompt})           # append current user prompt — last message before inference

            # 6. Stream from GCE
            self.get_logger().info(f"Messages sent to GCE: {messages}")
            cognitive_response = await self._stream_gce(messages)               # stream GCE response — publishes chunks as they arrive

            # 7. Register assistant turn in memory
            if cognitive_response:                                              # only register non-empty responses — empty means GCE failed
                await self.mcc.register_memory("assistant", cognitive_response) # bind assistant response into WMC — completes the PMT pair

            # 8. Report memory stats
            self.mcc.report_memory_stats()                                      # log WMC and EMC health after every turn

        except Exception as e:                                                  # unhandled failure in cognitive pipeline
            self.get_logger().error(f"❌ CNC handle error: {e}")                # log failure with reason
            self._emit_response({"type": "error", "content": str(e)})           # surface error to caller

        finally:                                                                # always runs — resets attention gate regardless of success or failure
            self._attention_gate = False                                        # reopen attentional gate — ready for next stimulus

    async def _stream_gce(self, messages: list[dict]) -> str:
        """
        Stream GCE response via OpenAI-compatible API.
        Publishes chunks to ROS2 topic as they arrive.

        Returns:
            str: Full concatenated response string.
        """
        payload = {
            "model"              : GCE.COGNITIVE_ENGINE,
            "messages"           : messages,
            "max_tokens"         : GCE.RESPONSE_DEPTH,
            "temperature"        : GCE.TEMPERATURE,
            "top_p"              : GCE.PROBABILITY_THRESHOLD,
            "top_k"              : GCE.CANDIDATE_THRESHOLD,
            "repetition_penalty" : GCE.PERSEVERATION_DAMPING,
            "frequency_penalty"  : GCE.HABITUATION_DAMPING,
            "presence_penalty"   : GCE.NOVELTY_BIAS,
            "stream"             : True,
        }

        cognitive_response = ""
        is_first      = True

        try:
            async with self._gce_gateway.stream(
                "POST",
                "/v1/chat/completions",
                json=payload,
            ) as resp:

                if resp.status_code != 200:
                    err = f"GCE HTTP {resp.status_code}"
                    self.get_logger().error(f"❌ {err}")
                    self._emit_response({"type": "error", "content": err})
                    return ""

                async for line in resp.aiter_lines():
                    if not line or not line.startswith("data:"):
                        continue

                    data_str = line[len("data:"):].strip()
                    if data_str == "[DONE]":
                        break

                    try:
                        chunk = json.loads(data_str)
                    except json.JSONDecodeError:
                        continue

                    delta = (
                        chunk.get("choices", [{}])[0]
                        .get("delta", {})
                        .get("content", "")
                    )
                    if not delta:
                        continue

                    cognitive_response += delta

                    if is_first:
                        self._emit_response({"type": "start", "content": delta})
                        is_first = False
                    else:
                        self._emit_response({"type": "delta", "content": delta})

            if cognitive_response:
                self._emit_response({"type": "done", "content": cognitive_response})
                self.get_logger().info(f"✅ Response: {len(cognitive_response)} chars")
            else:
                self._emit_response({"type": "error", "content": "Empty response from GCE"})

        except httpx.TimeoutException:
            err = "GCE timeout — model may still be loading"
            self.get_logger().error(f"❌ {err}")
            self._emit_response({"type": "error", "content": err})

        except Exception as exc:
            self.get_logger().error(f"❌ Stream error: {exc}")
            self._emit_response({"type": "error", "content": str(exc)})

        return cognitive_response

    def _emit_response(self, payload: dict):
        """Publish a JSON payload to the cognitive response topic."""
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self._cognitive_response.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"❌ Publish error: {exc}")

    def destroy_node(self):
        """Clean shutdown — close memory and HTTP client."""
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

    async def _prime_gce(self) -> None:
        """Warm up GCE — load model into VRAM and pin it for the session."""
        try:
            payload = {
                "model"    : GCE.COGNITIVE_ENGINE,
                "messages" : [{"role": "user", "content": "hi"}],
                "max_tokens": 1,                                            # minimal response — just enough to trigger model load
                "stream"   : False,
            }
            if GCE.KEEP_ALIVE is not None:                                  # Ollama only — vLLM keeps models loaded permanently
                payload["keep_alive"] = GCE.KEEP_ALIVE                     # pin model in VRAM for session duration
            await self._gce_gateway.post("/v1/chat/completions", json=payload)
            self.get_logger().info("✅ GCE warmed up — model pinned in VRAM")
        except Exception as e:
            self.get_logger().warning(f"⚠️ GCE warmup failed: {e}")         # non-fatal — model loads on first real request

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