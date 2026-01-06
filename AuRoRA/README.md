flowchart TD
    %% Define nodes with boxes
    subgraph PumpNode["Pump"]
        direction TB
        PumpText["Collect raw sensor data (CPU/GPU/etc)\nResponsibilities:\n- 3-level polling (HI/MED/LO)\n- Provide latest snapshot\n- No filtering or regulation"]
    end

    subgraph RegNode["Regulator"]
        direction TB
        RegText["Filter & Normalize, Control Heartbeat\nResponsibilities:\n- Clean & normalize data\n- Control snapshot rate (1-3Hz)\n- Ensure stable downstream data"]
    end

    subgraph OscNode["Oscillator"]
        direction TB
        OscText["Encode & Pack into protobuf\nResponsibilities:\n- Pack snapshot into protobuf\n- Publish data to server\n- Ensure format correctness"]
    end

    subgraph CoordNode["Coordinator"]
        direction TB
        CoordText["Manage traffic & monitor system health\nResponsibilities:\n- Control incoming/outgoing flow\n- Check RTT & error rate\n- Fail-safe: cut linkage if error\n- Ensure system reliability"]
    end

    %% Draw arrows
    PumpNode --> RegNode
    RegNode --> OscNode
    OscNode --> CoordNode
    CoordNode -- Feedback --> OscNode
