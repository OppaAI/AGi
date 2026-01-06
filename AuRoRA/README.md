```mermaid
flowchart LR
    OS[(Jetson OS & Sensors)] --> P

    subgraph PROC_SYSTEM [VTC PROC System Architecture]
        direction TB
        P["<b>P</b>ump\n<i>Data Ingress</i>\nPolling: HI/MED/LO"] --> R
        R["<b>R</b>egulator\n<i>Slew-Rate Governor</i>\nHeartbeat Rhythm: 60-200 OPM"] --> O
        O["<b>O</b>scillator\n<i>Reference Clock</i>\nEncodes & Publishes"] --> C
        C["<b>C</b>oordinator\n<i>Safety Interlock</i>\nMonitors RTT/Error Rate"] -- "Health Report\n(RTT/Error Rate)" --> R
        C -- "Circuit Breaker\n(is_connected)" --> R
    end

    C <==>|UDP/ROS2 Binary| S((Server / Vital Pulse Analyzer))

    style P fill:#f4f7fc,stroke:#4a90e2,stroke-width:2px
    style R fill:#fff9f0,stroke:#f5a623,stroke-width:2px
    style O fill:#f0fff4,stroke:#2ecc71,stroke-width:2px
    style C fill:#fff0f0,stroke:#e74c3c,stroke-width:4px
    style S fill:#eee,stroke:#333




