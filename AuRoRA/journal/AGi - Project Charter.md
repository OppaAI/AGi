# Amazing Grace infrastructure (AGi) — Project Charter

## 📝 1. Executive Summary

AGi (Amazing GRACE Infrastructure) is a long-term personal robotics project
aimed at building GRACE — a humanistic, embodied AI companion capable of
assisting with daily life, perceiving and interacting with the physical world,
and growing smarter through accumulated experience. The project is led by a
single independent developer and built entirely on open-source and edge AI
technologies. GRACE is designed around a full cognitive stack including
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
that gap — starting with GRACE as a proof of concept and reference
architecture for what a truly personal AI companion can be.

### 3.2 Impact

GRACE's successful development demonstrates that a single independent
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
physical environments. AGi positions GRACE at this frontier,
implementing neuroscience-inspired cognitive architecture on accessible
consumer hardware.

**Humanistic AI Design:** There is growing recognition that AI systems
interacting closely with people must be designed with ethical awareness,
emotional sensitivity, and transparent behavior. GRACE's conscience
module and humanistic naming conventions reflect this principle as a
foundational design decision, not a feature addition.

## 🔭 4. Vision Statement 

AGi envisions a future where embodied AI companions are not just tools
but genuine presences — minds that perceive the world, accumulate
knowledge, and grow through experience. GRACE represents the first
realization of this vision: a robot that can move through the world
alongside a person, notice what they notice, remember what they've
shared, and engage with the environment as a curious, feeling
participant.

To make that possible, GRACE is being built with a full cognitive stack:
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

AGi exists to build GRACE: a humanistic robotic companion capable of
assisting with daily life, exploring the world, and growing smarter over
time. The mission is to give GRACE the cognitive architecture — memory,
knowledge, conscience, perception, and feeling — to be as present and
interactive as possible with her environment, her surroundings, and the
people she's with.

## 📦 6. Scope

### 6.1 Objectives

- Build GRACE as a fully functional humanistic embodied AI companion
  running on edge hardware
- Implement a complete cognitive architecture including memory, knowledge,
  conscience, perception, and reasoning
- Develop agentic capability so GRACE can take actions, use tools, and
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
- **Agentic Tool Use** — GRACE can autonomously call external tools and
  APIs to complete tasks (weather, search, forecasts, reminders, and
  more)
- **Conscience Module** — Ethical reasoning and behavioral guardrails
  embedded as a core cognitive layer
- **Perception Stack** — Vision, audio, and sensory modules enabling
  real-time environmental awareness
- **Knowledge Base** — Internet-connected knowledge acquisition and
  structured long-term storage for self-directed learning
- **Chatbot Interface** — Conversational interface for natural
  interaction with GRACE
- **Hardware Integration** — Full ROS2 stack running on NVIDIA Jetson
  Orin Nano with sensor and motor control

### 6.3 Boundaries / Non-Scope

The following are intentionally excluded from the current scope of AGi
and are deferred to future phases or left to the broader community:

- **Cloud dependency** — GRACE is designed for edge inference and local
  operation. Cloud-based AI services are not a target architecture
- **Multi-robot coordination** — AGi focuses on a single GRACE instance.
  Fleet management or multi-agent coordination is out of scope
- **Commercial productization** — AGi is an independent open-source R&D
  project. Packaging, licensing, or commercializing GRACE as a consumer
  product is not a current objective
- **Full humanoid platform** — Humanoid hardware is the long-term
  hardware vision but is outside the current build scope
- **Formal safety certification** — GRACE is a research platform and is
  not being built to any regulatory or industrial safety standard
- **Third-party integrations beyond core tools** — Deep integration with
  external platforms beyond GRACE's core agentic tools is deferred until
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
| OppaAI                | Successful realization of GRACE as a humanistic AI companion                     |
| Open-Source Community | Access to a reference architecture for embodied humanistic AI                    |
| General Public        | A generalist AI companion that simplifies and enriches daily life                |

