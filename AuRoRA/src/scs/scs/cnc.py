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
    Sub: SCS.TEXT_INPUT_GATEWAY (std_msgs/String)   — incoming text input signal
    Pub: SCS.RESPONSE_GATEWAY   (std_msgs/String)   — streamed cognitive response
    Pub: SCS.MEMORY_CONTEXT_GATEWAY (std_msgs/String) — full GCE input context for debug
    Pub: SCS.MEMORY_STATS_GATEWAY (std_msgs/String) — memory cortex stats after every turn

Response format (JSON on SCS.RESPONSE_GATEWAY):
    {"type": "start", "content": "<first fragment>"}
    {"type": "delta", "content": "<fragment>"}
    {"type": "done",  "content": "<full cognitive response>"}
    {"type": "error", "content": "<error message>"}

Terminology:
    Gamma Rhythm    — asyncio loop coordinating active cognition across memory and inference
    Neural Gateway  — persistent async HTTP connection to an external cognitive engine
    Neural Thread   — dedicated background thread hosting an async event loop
    Thalamic Relay  — role of the CNC in routing signals between sensory input, memory systems, and motor output.
"""

# System components
import asyncio                                             # dedicated event loop for async memory and inference operations — runs on cnc-asyncio thread
from concurrent.futures import ThreadPoolExecutor          # thread pool for offloading blocking operations from the ROS2 spin thread
from datetime import datetime                              # for injecting current date into system prompt each turn
import httpx                                               # async HTTP client for GCE streaming — OpenAI-compatible SSE
import json                                                # for serializing ROS2 message payloads and parsing GCE SSE chunks
import threading                                           # for cnc-asyncio background thread hosting the event loop

# ROS2 components
import rclpy                                               # ROS2 Python client library — node lifecycle and spin
from rclpy.node import Node                                # base class for all ROS2 nodes
from rclpy.executors import MultiThreadedExecutor          # allows concurrent callback execution — required for async scheduling
from std_msgs.msg import String                            # ROS2 string message type for text I/O

# AGi components
from hrs.hrm import AGi                                    # homeostatic regulation manifest namespace
from hrs.hru import hydrate_manifest, UserAccessLevel      # manifest hydration + + user type enum— binds AuRoRA parameter server values into AGi constants at node init
from scs.mcc import MemoryCoordinationCore                 # memory coordinator — CNC never touches WMC or EMC directly
from scs.niu import NeuralStimulus, NeuralInputChannel     # neural input channel— input gateway for all neural stimuli (text, speech, etc.)
from scs.iru import IdentityRecognitionUnit                # identity recognition unit — identify user and establish session identity and user context
from scs.ppu import PersonalProgressionUnit                # Personal Provisioning Unit — session identity and user context loader

SCS = AGi.SCS                                              # module-level alias — SCS-level constants (topic names, cortical capacity)
GCE = AGi.SCS.GCE                                          # module-level alias — GCE constants (model, endpoint, inference parameters)

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
            - Reticular Activating Compartment (RAC) — activation reaction that hydrates AGi and spawns all nodes
            - Generative Cognitive Engine (GCE) — persistent connection for inference of generative cognition
            - Memory Coordination Cortex (MCC) — short-term and long-term memory management
            - Neural threads — dedicated asyncio loops for parallel operations
            - Neural gateways — ROS2 topics for text input and cognitive output


        """
        super().__init__("cnc")                                             # register this node with ROS2 as "cnc"
        self.get_logger().info("=" * 60)                                    # visual separator — stdout and /rosout
        self.get_logger().info("🧠 CNC — Central Neural Core starting…")    # boot announcement — stdout and /rosout
        self.get_logger().info("=" * 60)                                    # visual separator — stdout and /rosout

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

        # Initialize neural gateway for text input, generative cognitive output and MCC stats
        self._text_input_stimulus: rclpy.subscription.Subscription = self.create_subscription(  # ROS2 subscriber — fires on every incoming message
            String, SCS.TEXT_INPUT_GATEWAY, self._receive_text_input, 10                        # String type | topic | callback | QoS depth 10
        )
        self._cognitive_response: rclpy.publisher.Publisher = self.create_publisher( # ROS2 publisher — sends cognitive output to the specified topic
            String, SCS.RESPONSE_GATEWAY, 10                                         # String type | topic | QoS depth 10
        )
        self._memory_context_feedback: rclpy.publisher.Publisher = self.create_publisher( # ROS2 publisher — sends memory context to the specified topic
            String, SCS.MEMORY_CONTEXT_GATEWAY, 10                                        # String type | topic | QoS depth 10
        )
        self._memory_stats_feedback: rclpy.publisher.Publisher = self.create_publisher(    # ROS2 publisher — sends memory stats to the specified topic
            String, SCS.MEMORY_STATS_GATEWAY, 10                                           # String type | topic | QoS depth 10
        )

        # Initialize attentional gate — True while processing a turn, drops incoming stimuli
        self._attention_gate: bool = False                                  # attentional gate — True while processing a turn, drops incoming stimuli
        self._pending_stimulus: str | None = None                           # single-slot busy queue — holds one stimulus while CNC is processing
        asyncio.run_coroutine_threadsafe(                                   # submit GCE priming to cognitive cycle — fire and forget, doesn't block init
            self._prime_gce(), self._cognitive_cycle                        # schedules across thread boundary — safe from ROS2 main thread
        )

        self.get_logger().info(f"✅ Endpoint    : {GCE.NEURAL_ENDPOINT}")                   # confirm GCE endpoint
        self.get_logger().info(f"✅ Model       : {self._gce_inference_packet['model']}")   # confirm GCE model
        self.get_logger().info(f"✅ Subscribed  : {SCS.TEXT_INPUT_GATEWAY}")                # confirm input topic
        self.get_logger().info(f"✅ Publishing  : {SCS.RESPONSE_GATEWAY}")                  # confirm output topic
        self.get_logger().info("=" * 60)                                                     # visual separator
        self.get_logger().info("🌸 GRACE is ready")                                         # boot complete
        self.get_logger().info("=" * 60)                                                     # visual separator
   
    def _receive_text_input(self, msg: String) -> None:
        """
        Receive an incoming text input signal and schedule cognitive processing.
        Queues one input signal if cognitive engine is already processing a previous user input.
        Single-slot, latest-wins semantics: if queue is occupied, newest stimulus overwrites pending.
        
        Args:
            msg (String): ROS2 string message carrying the incoming text input signal
        """
        try:                                                                                                    # attempt to parse against NeuralStimulus contract
            raw_input: dict = json.loads(msg.data.strip())                                                      # deserialize JSON payload — all input sources send JSON
            if not isinstance(raw_input, dict) or not raw_input.get("text"):                                    # malformed or missing text field — reject at boundary
                return                                                                                          # abort early
            ui_input = NeuralStimulus(                                                                          # parse against typed contract — enforces field presence
                text    = raw_input["text"].strip(),                                                            # user message content
                user_id = raw_input.get("user_id", "demo"),                                                     # speaker identity — defaults to demo if omitted
                source  = NeuralInputChannel(raw_input .get("source", NeuralInputChannel.CLI.value)),           # input modality — defaults to CLI if omitted
            )
        except (json.JSONDecodeError, ValueError):                                                              # malformed JSON or invalid NeuralInputChannel value — reject at boundary
            return                                                                                              # abort early — no plain string fallback, contract enforced

        if self._attention_gate:                                                                                # cognitive cycle busy — queue or overwrite pending stimulus
            if self._pending_stimulus is not None:                                                              # slot occupied — latest stimulus wins, previous discarded
                self.get_logger().warning("⚠️  Cognitive Engine queue overwritten — previous stimulus dropped") # log the queue is full and dropping preceding user input
            self._pending_stimulus = ui_input.text                                                              # overwrite with latest — most recent intent takes priority
            self.get_logger().info("⏳ Cognitive Engine is busy — stimulus queued")                             # log the cognitive cycle congestion
            return                                                                                              # stimulus queued — cognitive cycle will drain on turn completion
            
        self._attention_gate = True                                                                             # close gate before scheduling — prevents TOCTOU
        asyncio.run_coroutine_threadsafe(                                                                       # schedule cognitive pipeline — crosses thread boundary safely
            self._process_stimulus(ui_input.text), self._cognitive_cycle                                        # submit to gamma rhythm — never blocks ROS2 spin
        )

    async def _process_stimulus(self, user_prompt: str) -> None:
        """
        Process one full stimulus-response cycle.
    
        Registers the user prompt in memory, assembles the full memory context
        (WMC PMTs + recalled EMC episodes), builds the system prompt with episodic
        context injected, streams inference through GCE, registers the assistant
        response in memory, then reports memory stats.
        """
        cognitive_response: str = ""                                                              # accumulates GCE response chunks — empty until inference completes

        try:                                                                                      # wrap full pipeline — any failure publishes error and resets attention gate
            # 1. Register user turn in memory
            await self.mcc.register_memory(                                                       # induce user turn into WMC — evicted PMTs bound to EMC asynchronously
                role="user",
                user_id=self._active_user,
                content=user_prompt
            )

            # 2. Assemble memory context
            memory_context: list[dict] = await self.mcc.assemble_memory_context(                  # assemble full memory context — EMC episodes + WMC PMTs
                user_id=self._active_user if self._access_level == UserAccessLevel.GUEST else None,
                user_prompt=user_prompt,
            )

            # 3. Separate system and conversation parts
            long_term_memory: list[dict] = [m for m in memory_context if m["role"] == "system"]   # extract system blocks
            short_term_memory: list[dict]  = [m for m in memory_context if m["role"] != "system"] # extract conversation turns

            # 4. Assemble system prompt with date and inject user preferences into system prompt
            active_cognition: str = self._ppu.active_cognition.format(                            # loads system prompt from PPU - which loads it from the persona YAML
                date=datetime.now().strftime("%Y-%m-%d")                                          # inject current date in ISO-8601 format
            )

            if long_term_memory:                                                                  # episodic context available — append to system prompt
                system_content: str = active_cognition + "\n\n" + "\n\n".join(m["content"] for m in long_term_memory)  # fuse personality + episodic context
            else:
                system_content: str = active_cognition                                            # no episodic context — personality prompt only

            # 5. Build final message list
            messages: list[dict] = [{"role": "system", "content": system_content}]                # system prompt — always first
            messages.extend(short_term_memory)                                                    # inject WMC PMTs — chronological conversation history
            messages.append({"role": "user", "content": user_prompt})                             # append current user prompt — last message before inference
            
            context_signal = String()                                                             # create ROS2 String message
            context_signal.data = json.dumps({"messages": messages})                              # serialize payload to JSON string
            self._memory_context_feedback.publish(context_signal)                                 # publish to memory context reporting topic

            # 6. Stream from GCE
            self.get_logger().debug(f"📤 GCE messages: {messages}")                               # debug — full message context before inference
            cognitive_response = await self._stream_gce(messages)                                 # stream GCE response — publishes chunks as they arrive

            # 7. Register assistant turn in memory
            if cognitive_response:                                                                # only register non-empty responses — empty means GCE failed
                await self.mcc.register_memory(role="assistant", user_id=None, content=cognitive_response) # bind assistant response into WMC — completes the PMT pair

            # 8. Report memory stats
            stats = self.mcc.report_memory_stats()                                                # log WMC and EMC health after every turn
            stats_signal = String()                                                               # create ROS2 String message
            stats_signal.data = json.dumps(stats)                                                 # serialize payload to JSON string
            self._memory_stats_feedback.publish(stats_signal)                                     # publish to memory stats reporting topic
            
        except Exception as e:                                                                    # unhandled failure in cognitive pipeline
            self.get_logger().error(f"❌ CNC handle error: {e}")                                  # log failure with reason
            self._emit_response({"type": GCE.STREAM_ANOMALY, "content": str(e)})                  # surface error to caller

        finally:                                                                                  # always runs — resets attention gate regardless of success or failure
            self._attention_gate = False                                                          # reopen attentional gate — ready for next stimulus
            if self._pending_stimulus:                                                            # queued stimulus present — drain immediately
                pending = self._pending_stimulus                                                  # snapshot before clearing — prevents TOCTOU
                self._pending_stimulus = None                                                     # clear slot before scheduling — gate reopens clean
                self._attention_gate = True                                                       # close gate before scheduling — queued stimulus now owns the cycle
                asyncio.ensure_future(self._process_stimulus(pending))                            # schedule queued stimulus on the running cognitive cycle — already on the loop, no thread crossing needed
    
    async def _stream_gce(self, messages: list[dict]) -> str:
        """
        Generate and emit a cognitive response through the Generative Cognitive Engine.
        Response is streamed as fragments to the cognitive output gateway as it arrives.

        Args:
            messages (list[dict]): Full memory context and current user prompt for inference.

        Returns:
            str: Full cognitive response for memory registration.
        """

        cognitive_response: str = ""                                        # accumulates response fragments into full response
        leading_fragment: bool = True                                       # tracks first fragment — triggers start event type
        self._gce_inference_packet["messages"] = messages                   # full memory context + current user prompt
        try:                                                                # attempt to forward request to GCE and stream response fragments
            async with self._gce_gateway.stream(                            # open streaming HTTP connection to GCE
                "POST",                                                     # POST request method for SSE streaming
                "/v1/chat/completions",                                     # SSE endpoint — matches OpenAI-compatible API
                json=self._gce_inference_packet,                            # JSON body with inference parameters
            ) as response:                                                  # asynchronous response stream

                if response.status_code != 200:                             # non-200 — GCE rejected the request
                    anomaly: str = f"GCE HTTP {response.status_code}"       # build error message with status code
                    self.get_logger().error(f"❌ {anomaly}")                # log failure with reason
                    self._emit_response({"type": GCE.STREAM_ANOMALY, "content": anomaly})  # surface HTTP error to caller
                    return ""

                async for line in response.aiter_lines():                   # iterate SSE stream line by line
                    if not line or not line.startswith("data:"):            # skip empty lines and non-data SSE events
                        continue

                    fragment_data: str = line[len("data:"):].strip()        # strip "data:" prefix — extract JSON payload
                    if fragment_data == "[DONE]":                           # SSE stream complete signal
                        break

                    try:                                                    # attempt to parse SSE fragment
                        fragment: dict = json.loads(fragment_data)          # parse SSE fragment — each fragment is a JSON object
                    except json.JSONDecodeError:                            # malformed JSON — skip this fragment
                        continue                                            # skip and continue stream

                    fragment_content: str = (                               # extract delta from SSE fragment
                        fragment.get("choices", [{}])[0]                    # first choice in the choices list
                        .get(GCE.STREAM_PROPAGATING, {})                    # token "delta" from fragment
                        .get("content", "")                                 # text content — empty string if no content in this fragment
                    )
                    if not fragment_content:                                # no content in this fragment — skip
                        continue                                            # skip and continue stream

                    cognitive_response += fragment_content                  # accumulate fragment into full response

                    if leading_fragment:                                    # first fragment — signal stream start to caller
                        self._emit_response({"type": GCE.STREAM_LEADING, "content": fragment_content})  # publish "start" event
                        leading_fragment = False                            # clear flag after first fragment
                    else:                                                   # subsequent fragments — signal token arrival
                        self._emit_response({"type": GCE.STREAM_PROPAGATING, "content": fragment_content})  # subsequent fragments — stream "delta" to caller

            if cognitive_response:                                                            # response assembled — signal completion
                self._emit_response({"type": GCE.STREAM_TRAILING, "content": cognitive_response}) # publish "done" event
                self.get_logger().info(f"✅ Response: {len(cognitive_response)} chars")       # log completion with response length
            else:                                                                             # no response assembled — publish "error" event
                self._emit_response({"type": GCE.STREAM_ANOMALY, "content": "Empty response from GCE"})  # GCE returned nothing

        except httpx.TimeoutException:                                      # GCE exceeded timeout — model may still be loading
            err = "GCE timeout — model may still be loading"                # timeout error message
            self.get_logger().error(f"❌ {err}")                            # log timeout error with reason
            self._emit_response({"type": GCE.STREAM_ANOMALY, "content": err}) # publish timeout "error" to caller

        except Exception as e:                                               # unhandled stream errors
            self.get_logger().error(f"❌ Stream error: {e}")                # log unexpected error
            self._emit_response({"type": GCE.STREAM_ANOMALY, "content": str(e)})     # publish "error" to caller

        return cognitive_response                                           # return accumulated response

    async def _prime_gce(self) -> None:
        """
        Prime GCE — activate the cognitive engine and pin it for the session.
        Fire and forget — called once at boot, non-blocking.
        """
        try:                                                                # attempt to preload GCE model into memory
            inference_packet: dict = {                                      # minimal inference request — triggers model load into VRAM
                "model"    : GCE.COGNITIVE_ENGINE,                          # GCE model to prime
                "messages" : [{"role": "user", "content": "hi"}],           # minimal prompt — just enough to trigger model load
                "max_tokens": 1,                                            # single token response — minimizes priming cost
                "stream"   : False,                                         # no streaming needed for priming
            }
            await self._gce_gateway.post("/v1/chat/completions", json=inference_packet)  # submit priming request to GCE
            self.get_logger().info("✅ GCE primed successfully — activated into memory")  # log the successful activation of GCE
        except Exception as e:
            self.get_logger().warning(f"⚠️ GCE priming failed: {e}")         # non-fatal — model loads on first real request
            
    def _emit_response(self, response_packet: dict) -> None:
        """
        Emit a cognitive response fragment to the ROS2 output topic.
    
        Args:
            response_packet (dict): Response payload — type and content fields.
        """
        try:                                                                      # attempt to serialize and publish response packet
            raw_signal: String = String()                                         # create ROS2 String message
            raw_signal.data = json.dumps(response_packet)                         # serialize payload to JSON string
            self._cognitive_response.publish(raw_signal)                          # publish to cognitive response topic
        except Exception as e:                                                  
            self.get_logger().error(f"❌ Publish error: {e}")                     # non-fatal — publishing failure must not crash the cognitive cycle
   
    def destroy_node(self) -> None:
        """
        Gracefully shut down CNC — close all cognitive subsystems in reverse boot order.
        """
        self.get_logger().info("🛑 CNC shutting down…")                            # announce shutdown sequence — stdout and /rosout
        self.mcc.close()                                                            # close memory coordination — EMC encoding cycle stops

        future = asyncio.run_coroutine_threadsafe(                                  # schedule HTTP client close on cognitive cycle
            self._gce_gateway.aclose(), self._cognitive_cycle                       # async close — releases TCP connections
        )
        try:                                                                        # attempt clean close — best-effort on shutdown
            future.result(timeout=3.0)                                              # wait up to 3 seconds for clean close
        except Exception:
            pass                                                                    # ignore — process is ending, clean close is best-effort

        self._cognitive_cycle.call_soon_threadsafe(self._cognitive_cycle.stop)      # signal cognitive cycle to stop
        self._gamma_rhythm.join(timeout=3.0)                                        # wait for gamma rhythm thread to exit
        self._cognitive_executor.shutdown(wait=True)                                # drain thread pool — wait for pending tasks

        super().destroy_node()                                                      # ROS2 node cleanup
        self.get_logger().info("✅ CNC shutdown complete")                          # log the shutdown sequence complete

def main(args=None):
    rclpy.init(args=args)                                        # initialize ROS2 context
    node = CNC()                                                 # instantiate CNC node — boots all subsystems
    executor = MultiThreadedExecutor()                           # concurrent callback execution
    executor.add_node(node)                                      # register CNC with executor
    try:
        executor.spin()                                          # block until shutdown — processes ROS2 callbacks
    except KeyboardInterrupt:                                    # user press Ctrl-C interrupt
        node.get_logger().info("👋 Shutdown requested")          # log the graceful shutdown request
    finally:                                                     # always runs — ensure clean shutdown regardless of interrupt
        executor.shutdown()                                      # stop executor
        node.destroy_node()                                      # clean shutdown sequence
        rclpy.shutdown()                                         # release ROS2 context

if __name__ == "__main__":
    main()
