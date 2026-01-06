```mermaid
flowchart LR
    %% External Inputs
    OS[(Jetson OS &<br/>Sensors)] --> P

    subgraph PROC_SYSTEM [PROC System Architecture]
        direction TB
        P[<b>P</b>ump<br/><i>Data Ingress</i>] --> R
        
        R[<b>R</b>egulator<br/><i>Slew-Rate Governor</i>] --> O
        
        O[<b>O</b>scillator<br/><i>Reference Clock</i>] --> C
        
        C[<b>C</b>oordinator<br/><i>Safety Interlock</i>] -- "Health Report<br/>(RTT/Error Rate)" --> R
        
        C -- "Circuit Breaker<br/>(is_connected)" --> R
    end

    %% External Output
    C <==>|UDP/ROS2 Binary| S((Server / Vital<br/>Pulse Analyzer))

    %% Styling
    style P fill:#f4f7fc,stroke:#4a90e2,stroke-width:2px
    style R fill:#fff9f0,stroke:#f5a623,stroke-width:2px
    style O fill:#f0fff4,stroke:#2ecc71,stroke-width:2px
    style C fill:#fff0f0,stroke:#e74c3c,stroke-width:4px
    style S fill:#eee,stroke:#333
