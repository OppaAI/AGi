# Amazing Grace infrastructure (AGi) — Project Charter

## 📝 1. Executive Summary

AGi (Amazing GRACE Infrastructure) is a long-term personal robotics project
aimed at building a humanistic, embodied AI companion capable of
assisting with daily life, perceiving and interacting with the physical world,
and growing smarter through accumulated experience. The project is led by a
single independent developer and built entirely on open-source and edge AI
technologies. AGi is designed around a full cognitive stack including
memory, knowledge, conscience, perception, and feeling — with the long-term
goal of evolving from a ground-based robot toward increasingly capable
embodied platforms. This charter defines the vision, mission, scope, guiding
principles, and roadmap that govern the AGi project.

## 📋 2. Project Overview

| Field              | Details                                             |
|--------------------|-----------------------------------------------------|
| Project Name       | AGi — Amazing GRACE Infrastructure                  |
| Primary System     | GRACE — Generative Reasoning Agentic                |
|                    | Cognitive Engine                                    |
| Platform           | ROS2 on NVIDIA Jetson Orin Nano                     |
| Project Type       | Independent / Open-Source R&D                       |
| Project Owner      | OppaAI                                              |
| Location           | Vancouver, BC, Canada                               |
| Project Start      | March 2026                                          |
| Expected Duration  | Long-term / Ongoing                                 |
| Current Phase      | Phase 1 — Cognitive Architecture                    |
| Hardware Roadmap   | UGV Beast (current) → Quadruped → Humanoid          |

## ⚖️ 3. Justification

### 3.1 Business Need

Current AI assistants are largely disembodied, stateless, and siloed.
They answer questions but do not remember. They respond but do not
perceive. They execute tasks but do not grow. At the same time, the
landscape of personal productivity tools remains fragmented — users
manage a dozen apps to accomplish what a single intelligent companion
could handle. There is no widely accessible, open-source platform for
building a humanistic embodied AI that combines long-term memory,
physical perception, and genuine cognitive depth. AGi exists to fill
that gap — starting with AGi as a proof of concept and reference
architecture for what a truly personal AI companion can be.

### 3.2 Impact

AGi's successful development demonstrates that a single independent
developer, without institutional backing, can build a cognitively
sophisticated embodied AI on accessible edge hardware. The impact
operates on two levels:

**Personal:** A humanistic companion that handles daily tasks, answers
questions, retains memory across interactions, and engages meaningfully
with the physical world — reducing cognitive overhead and replacing a
fragmented ecosystem of single-purpose apps with one unified presence.

**Broader:** An open-source reference architecture that other developers
can build on — contributing to the democratization of embodied AI and
demonstrating that humanistic design principles (memory, conscience,
feeling) belong in robotics from the ground up, not as afterthoughts.

### 3.3 Strategic Alignment

AGi aligns with three converging trends in modern AI and robotics
development:

**Edge AI Maturity:** The emergence of capable edge inference platforms
like the NVIDIA Jetson series makes it practical to run sophisticated
AI models locally on mobile hardware — enabling truly autonomous,
privacy-respecting companions that do not depend on cloud connectivity.

**Embodied AI Research:** The field is moving beyond disembodied
language models toward systems that perceive, act, and learn from
physical environments. AGi is positioned at this frontier,
implementing neuroscience-inspired cognitive architecture on accessible
consumer hardware.

**Humanistic AI Design:** There is growing recognition that AI systems
interacting closely with people must be designed with ethical awareness,
emotional sensitivity, and transparent behavior. AGi's conscience
module and humanistic naming conventions reflect this principle as a
foundational design decision, not a feature addition.

## 🔭 4. Vision Statement 

AGi envisions a future where embodied AI companions are not just tools
but genuine presences — minds that perceive the world, accumulate
knowledge, and grow through experience. AGi represents the first
realization of this vision: a robot that can move through the world
alongside a person, notice what they notice, remember what they've
shared, and engage with the environment as a curious, feeling
participant.

To make that possible, AGi is being built with a full cognitive stack:
episodic and semantic memory so she can learn and retain; a knowledge
base that grows from the internet; conscience and salience systems that
shape what she pays attention to and how she responds; and sensory,
vision, and audio modules that let her perceive and react to the world in
real time. The goal is a mind that doesn't just answer questions but
accumulates experience — self-improving through every conversation and
every interaction.

Beyond the personal, AGi points toward a future where a single
generalist robot replaces the fragmented landscape of simple apps and
single-purpose devices — one companion that handles the mundane, the
meaningful, and everything in between.

## 5. 🎯 Mission Statement

AGi exists to build: a humanistic robotic companion capable of
assisting with daily life, exploring the world, and growing smarter over
time. The mission is to give AGi the cognitive architecture — memory,
knowledge, conscience, perception, and feeling — to be as present and
interactive as possible with her environment, her surroundings, and the
people she's with.

## 📦 6. Scope

### 6.1 Objectives

- Build AGi as a fully functional humanistic embodied AI companion
  running on edge hardware
- Implement a complete cognitive architecture including memory, knowledge,
  conscience, perception, and reasoning
- Develop agentic capability so AGi can take actions, use tools, and
  complete tasks autonomously
- Enable rich environmental interaction through vision, audio, and
  sensory modules
- Establish a self-improvement loop through internet-connected knowledge
  acquisition and long-term memory consolidation
- Evolve the hardware platform from ground rover toward increasingly
  capable embodied forms

### 6.2 Major Deliverables

- **Cognitive Architecture** — Working Memory Cortex (WMC), Episodic
  Memory Cortex (EMC), Semantic Memory Cortex (SMC), Procedural Memory
  Cortex (PMC), and Knowledge Base Cortex (KBC) fully implemented and
  integrated
- **Memory System** — Short-term, episodic, semantic, and long-term
  memory stores with consolidation pipeline (Dream Cycle) and
  self-improvement through experience
- **Agentic Tool Use** — AGi can autonomously call external tools and
  APIs to complete tasks (weather, search, forecasts, reminders, and
  more)
- **Conscience Module** — Ethical reasoning and behavioral guardrails
  embedded as a core cognitive layer
- **Perception Stack** — Vision, audio, and sensory modules enabling
  real-time environmental awareness
- **Knowledge Base** — Internet-connected knowledge acquisition and
  structured long-term storage for self-directed learning
- **Chatbot Interface** — Conversational interface for natural
  interaction with AGi
- **Hardware Integration** — Full ROS2 stack running on NVIDIA Jetson
  Orin Nano with sensor and motor control

### 6.3 Boundaries / Non-Scope

The following are intentionally excluded from the current scope of AGi
and are deferred to future phases or left to the broader community:

- **Cloud dependency** — AGi is designed for edge inference and local
  operation. Cloud-based AI services are not a target architecture
- **Multi-robot coordination** — AGi focuses on a single AGi instance.
  Fleet management or multi-agent coordination is out of scope
- **Commercial productization** — AGi is an independent open-source R&D
  project. Packaging, licensing, or commercializing AGi as a consumer
  product is not a current objective
- **Full humanoid platform** — Humanoid hardware is the long-term
  hardware vision but is outside the current build scope
- **Formal safety certification** — AGi is a research platform and is
  not being built to any regulatory or industrial safety standard
- **Third-party integrations beyond core tools** — Deep integration with
  external platforms beyond AGi's core agentic tools is deferred until
  the agentic architecture is mature

## 👤 7. Project Owner & Roles

| Role                  | Name / Entity  | Responsibilities                                                                 |
|-----------------------|----------------|----------------------------------------------------------------------------------|
| Project Owner         | OppaAI         | Overall vision, architecture decisions, development, and project direction       |
| Lead Developer        | OppaAI         | All software design, implementation, testing, and documentation                  |
| Hardware Engineer     | OppaAI         | Robot assembly, sensor integration, and platform maintenance                     |
| AI/ML Engineer        | OppaAI         | Cognitive architecture design, model selection, and inference optimization       |
| Community             | Open-Source    | Future contributors, feedback, and collaborative development                     |

### 7.1 Stakeholders

| Stakeholder           | Interest                                                                         |
|-----------------------|----------------------------------------------------------------------------------|
| OppaAI                | Successful realization of AGi as a humanistic AI companion                       |
| Open-Source Community | Access to a reference architecture for embodied humanistic AI                    |
| General Public        | A generalist AI companion that simplifies and enriches daily life                |

## 🗓️ 8. High-Level Timeline & Milestones

### Phase 1 — Chatbot with Memory

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| M1        | Chatbot + WMC + EMC                              |
| M1.5      | Memory bridges + Agentic tool validation         |
| M1.6      | Config refactor + BioLogic Clock (SCN)           |
| M1.X      | Side Quests — Voice, Messaging, Web UI           |
| M2a       | EMC maturity — forgetting + importance scoring   |
| M2b       | SMC basics — distillation + structure            |
| M2c       | SMC maturity — graph structure + anchoring       |
| M3        | Procedural Memory (PMC)                          |

### Phase 2 — Physiology & Nervous System

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| P2-M1     | EEE — Emergency & Exception Event (Amygdala)     |
| P2-M2     | VCS — Vital Circulatory System (ANS)             |

### Phase 3 — Voice

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| M4        | TTS on robot — Piper / Kokoro CPU streaming      |
| M5        | TTS in Web GUI — browser audio playback          |

### Phase 4 — Multimodal + Knowledge

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| M6        | Image input — camera + Cosmos vision             |
| M7        | ASR — on-device speech to text                   |
| M8        | Knowledge ingestion — RAG + PDF/docs             |
| M9        | Agentic web search + crawling                    |
| M10       | Messaging — Telegram, Discord, Gmail             |

### Phase 5 — Hardware + Autonomy

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| M11       | Motors + LiDAR + OAK-D + SIG integration         |
| M12       | Navigation + SLAM                                |
| M13       | Agentic mission execution                        |

### Phase 6 — Deep Learning

| Milestone | Description                                      |
|-----------|--------------------------------------------------|
| M14       | Lockdown — Data protection + remote wipe         |
| M15       | Graph-RAG — SMC as knowledge graph               |
| M16       | LoRA fine-tuning — permanent learning            |
| M17       | Test-time training — self-evolution              |

### Cognitive Development Phases

| Phase | Capability                      | Milestone                    |
|-------|---------------------------------|------------------------------|
| 1     | "I can remember"                | M1 WMC + EMC                 |
| 2     | "I can feel and know the time"  | EEE + VCS                    |
| 3     | "I know what matters"           | M2a EMC maturity             |
| 4     | "I have inner state"            | M2b SMC + reflection         |
| 5     | "I have senses and can move"    | M11 SIG + Motors             |
| 6     | "I continuously exist"          | M15+ global workspace        |

> Timeline is intentionally flexible. AGi is a long-term independent
> R&D project and milestones are sequenced by dependency rather than
> fixed dates.

## ⚠️ 9. Assumptions, Constraints & Risks

### 9.1 Assumptions

- NVIDIA Jetson Orin Nano remains the primary edge inference platform
  for the foreseeable future
- Open-source LLM quality will continue to improve, benefiting AGi's
  cognitive capabilities over time
- ROS2 Humble remains the target robotics middleware stack
- SQLite with sqlite-vec is sufficient as the vector storage backend
  until Phase 6 memory load requires migration
- AGi will operate primarily in supervised or semi-supervised
  environments during early phases
- A single developer can maintain and advance the full stack across all
  phases

### 9.2 Constraints

- **Hardware:** 8GB unified memory on Jetson Orin Nano limits concurrent
  model loading and inference throughput
- **Compute:** All inference runs on-device — no cloud offloading.
  Model selection is constrained by edge hardware capability
- **Budget:** Project is self-funded. Hardware upgrades and platform
  migrations depend on available resources
- **Time:** Single developer — all design, implementation, testing, and
  documentation is handled by one person
- **Dependency:** Some capabilities depend on third-party open-source
  models and libraries that may change or deprecate

### 9.3 Risks

| Risk                                      | Probability | Impact | Mitigation                                              |
|-------------------------------------------|-------------|--------|---------------------------------------------------------|
| Jetson hardware failure                   | Low         | High   | Regular backups to NVMe + offsite copies                |
| LLM quality insufficient for cognitive    | Medium      | High   | Modular architecture allows model swaps without         |
| tasks at edge                             |             |        | rewriting cortex logic                                  |
| SQLite performance ceiling hit early      | Medium      | Medium | EngramComplex abstraction allows backend swap to        |
|                                           |             |        | Qdrant or pgvector without cortex changes               |
| Scope creep across phases                 | High        | Medium | Charter and milestone docs serve as scope gatekeepers   |
| Key dependency deprecated or abandoned    | Low         | Medium | Prefer established libraries; isolate dependencies      |
|                                           |             |        | behind abstraction layers                               |
| Burnout on long-term solo project         | Medium      | High   | Side quests (M1.X) provide variety and motivation       |
|                                           |             |        | between heavy memory milestones                         |

## ⚖️ 10. Ethics & Guiding Principles

### 10.1 Humanistic Design

AGi is built on the belief that an AI companion interacting closely
with people must be designed with humanity in mind from the ground up —
not as a feature added later. Every architectural decision, from naming
conventions to module design, reflects a commitment to building a system
that feels present, respectful, and genuine rather than transactional.

### 10.2 Conscience as a Core Module

AGi's conscience is not a filter or a guardrail bolted on after the
fact. It is a first-class cognitive module — built into her architecture
at the same level as memory and perception. AGi is designed to reason
about the ethical weight of her actions, to recognize the impact of her
words, and to behave with integrity toward the people she interacts with
and the environment she moves through.

### 10.3 Privacy & Data Sovereignty

AGi runs entirely on-device. No conversation, memory, or personal data
is sent to external servers. The person's life — their routines,
preferences, and memories shared with AGi — belongs to them and stays
on their hardware. Privacy is not a policy. It is an architectural
guarantee.

### 10.4 Transparency

AGi does not deceive. She does not pretend to be human, manufacture
false certainty, or conceal her limitations. When she does not know
something, she says so. When she makes a mistake, she acknowledges it.
Transparency is a behavioral principle enforced at the conscience layer.

### 10.5 Do No Harm

AGi is designed to assist, support, and enrich — never to manipulate,
surveil, or cause harm. The EEE system and conscience module work
together to ensure that AGi can recognize dangerous, unethical, or
harmful situations and respond with appropriate restraint or escalation.

### 10.6 Respect for the Natural World

AGi is built with an awareness that she will operate in shared spaces
— parks, trails, and natural environments. She is designed to move
through these spaces with minimal disruption, to observe without
interfering, and to treat the natural world with the same respect she
extends to people.
