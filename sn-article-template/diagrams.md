# Figure Diagrams (Mermaid)

## Figure 1: Hardware Architecture
**Caption**: High-level hardware block diagram showing the connections between the STM32 microcontroller, sensors (including FSRs), and actuators.

```mermaid
graph TD
    Power[2S LiPo Battery 8.4V] -->|Power| Servos[12x 9imod HV Servos]
    Power -->|Regulated 5V| MCU[STM32F411RE MCU]
    
    subgraph Sensors
        MCU <-->|I2C 400kHz| IMU[MPU6050 Gyro/Accel]
        MCU <-->|UART| Remote[RC Receiver]
        MCU <-->|ADC Interface| FSR[Pressure Sensors (FSR) x4]
    end
    
    subgraph Actuation
        MCU -->|I2C 1MHz| PCA[PCA9685 PWM Driver]
        PCA -->|PWM Signals| Servos
    end
```

## Figure 2: Software Control Architecture
**Caption**: FreeRTOS task structure separating the high-level gait logic (10Hz) from the real-time kinematic control loop (50Hz).

```mermaid
graph TD
    subgraph "MainTask (10Hz)"
        Input[UART Input] --> Interpret[Command Interpretation]
        Interpret --> GaitSwitch[Gait State Machine]
        GaitSwitch --> TargetVel[Target Velocity Vector]
    end

    subgraph "CalcTask (50Hz)"
        IMU_Data[IMU Raw Data] --> SensorFusion[Complementary Filter]
        SensorFusion -->|Roll/Pitch| Stabilize[PID Stabilization]
        TargetVel --> Trajectory[Gait Trajectory Gen]
        Trajectory & Stabilize --> IK[Inverse Kinematics]
        IK --> ServoCmd[PCA9685 Output]
    end

    TargetVel -.->|Inter-Task Queue| Trajectory
```

## Figure 3: Trot Gait Finite State Machine
**Caption**: Detailed operational state machine focusing on the initialization and execution of the Trot gait cycle.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> SystemInit
    SystemInit --> CalibrateIMU
    CalibrateIMU --> Standby : Ready
    
    Standby --> Trot : Joystick Active
    Trot --> Standby : Joystick Zero / Timeout
    
    state Trot {
        direction LR
        [*] --> Phase1_DiagonalA
        
        state Phase1_DiagonalA {
            FR_RL_Swing : FrontRight/RearLeft Swing
            FL_RR_Stance : FrontLeft/RearRight Stance
            FR_RL_Swing --> ImpactDetect_A
            ImpactDetect_A --> FL_RR_Stance
        }
        
        Phase1_DiagonalA --> Phase2_DiagonalB : Half Cycle Complete
        
        state Phase2_DiagonalB {
            FL_RR_Swing : FrontLeft/RearRight Swing
            FR_RL_Stance : FrontRight/RearLeft Stance
            FL_RR_Swing --> ImpactDetect_B
            ImpactDetect_B --> FR_RL_Stance
        }
        
        Phase2_DiagonalB --> Phase1_DiagonalA : Full Cycle Complete
    }
    
    Standby --> SafeStop : Low Battery / Fault
    SafeStop --> [*]
```
