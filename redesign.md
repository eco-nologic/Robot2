# DrawRobot Redesign Plan: Phase 1 & 2 - Foundation & Communication

**Status**: Starting Fresh | **Target**: Working Robot with WiFi/BLE/Telemetry | **Approach**: Step-by-step, testing at each phase

---

## 🎯 Redesign Goals

✅ **Preserve Working Systems**:
- WiFi AP + WebSocket telemetry (CommsManager)
- BLE command bridge (BluetoothManager)
- Python remote control (robot_remote.py)
- Same file structure & class hierarchy

✅ **Fix & Rebuild**:
- Core motor control (IMotor → DCMotor)
- Encoder reading & odometry
- IMU integration
- Motion control loop

✅ **Implementation Strategy**:
- Start with **Phase 1**: Core foundation + motor control
- Move to **Phase 2**: Communication layer (reuse existing working code)
- Then: Sensors, motion planning, drawing sequences

---

## 📋 Implementation Phases

### **Phase 1: Foundation & Motor Control** (IN PROGRESS)

| Step | Component | File | Status |
|------|-----------|------|--------|
| **1.0** | Project Setup & Config | `Config.h` + `platformio.ini` | ✅ Complete |
| **1.1** | Motor Interface | `IMotor.h/cpp` | ✅ Complete |
| **1.2** | DC Motor Driver | `DCMotor.h/cpp` | ✅ Complete (PCNT encoder) |
| **1.3** | DriveTrain Control | `DriveTrain.h/cpp` | ✅ Complete (kinematics + odometry) |

**Deliverable**: Motors spin correctly, encoders read, robot moves forward/backward/turns ✅

---

### **Phase 1.5: Magnetometer Calibration & IMU Fusion** (CURRENT PRIORITY)

| Step | Component | File | Status |
|------|-----------|------|--------|
| **1.5.0** | Navigation & IMU | `Navigation.h/cpp` | 🔄 In Progress |
| **1.5.1** | Telemetry Packet | `TelemetryPacket.h` | 🔄 Ready |
| **1.5.2** | Magnetometer Hard-Iron Calibration | `Navigation.cpp` | 🔄 Implementing |
| **1.5.3** | Complementary Filter (Gyro+Mag) | `Navigation.cpp` | 🔄 Implementing |
| **1.5.4** | BLE Calibration Streaming | `BluetoothManager.cpp` | 🔄 To integrate |

**Deliverable**: Find true North, accurate heading, BLE telemetry streaming calibration progress to robot_remote.py

---

### **Phase 2: Communication Layer** (REUSE WORKING CODE)

| Step | Component | File | Status |
|------|-----------|------|--------|
| **2.0** | Battery Monitor | `BatteryMonitor.h/cpp` | ✅ Copy from PREVIOUS/ |
| **2.1** | Config Manager (NVS) | `ConfigManager.h/cpp` | ✅ Copy from PREVIOUS/ |
| **2.2** | Telemetry Packet | `TelemetryPacket.h` | 🔄 Create (X, Y, Heading, raw IMU, calibration state) |
| **2.3** | Bluetooth Manager (BLE) | `BluetoothManager.h/cpp` | 🔄 Copy from PREVIOUS/ |
| **2.4** | Comms Manager (WebSocket) | `CommsManager.h/cpp` | 🔄 Copy from PREVIOUS/ |
| **2.5** | Command Handler | `CommandHandler.h/cpp` | 🔄 Copy from PREVIOUS/ |

**Deliverable**: BLE + WiFi working, telemetry streaming (position + calibration), robot_remote.py connecting

---

### **Phase 3: Sensors & Fusion** (NEXT)

- Pose Estimator (PoseEstimator.h/cpp) - Fuse encoders + IMU heading
- IMU integration into telemetry
- Motion control loop (MotionController.h)

---

### **Phase 4: Drawing & Sequences** (LATER)

- Path planning (PathPlanner.h)
- Drawing sequences (CircleSequence, LineSequence, etc.)
- Pen control

---

## 🔧 Phase 1 Step-by-Step Implementation

### **Step 1.0: Configuration & Build System**

**Files to create/verify**:
1. `include/Config.h` - Hardware constants (pins, physics)
2. `platformio.ini` - Already configured ✅
3. `include/setup.h` - Initialization macros
4. `src/main.cpp` - Entry point skeleton

**Key decisions**:
- Keep all pin definitions from pin.pdf
- Real hardware only (DCMotor with ESP32 PCNT encoder reading)
- WiFi AP mode (robot creates network)

---

### **Step 1.1-1.4: Motor Control Pipeline**

```cpp
// Inheritance hierarchy:
IMotor (interface)
└── DCMotor (real hardware)

DriveTrain (uses two IMotor*)
├── setLinearSpeed(mm/s) → splits to left/right motors
├── setAngularSpeed(rad/s) → differential control
└── getPosition() → from encoders
```

**What to implement**:
1. **IMotor**: Pure virtual interface (begin, setPwm, setDirection, getEncoderCount)
2. **DCMotor**: PWM control + quadrature decoder for encoders
3. **DriveTrain**: Kinematics to convert (v, ω) → (pwm_left, pwm_right)

---

## 📁 File Structure (After Phase 1 & 2)

```
src/
├── main.cpp                      # Setup + main loop
├── IMotor.cpp                    # (Empty for interface)
├── DCMotor.cpp                   # Real motor control
├── DriveTrain.cpp                # Differential drive
├── Navigation.cpp                # IMU + magnetometer calibration
├── BatteryMonitor.cpp            # ADC voltage reading
├── ConfigManager.cpp             # NVS storage
├── BluetoothManager.cpp          # BLE service
├── CommsManager.cpp              # WebSocket server
├── CommandHandler.cpp            # Command parsing
└── [TelemetryPacket.h only]     # Header-only

include/
├── Config.h                      # 🔑 Hardware pins & constants
├── setup.h                       # Initialization helpers
├── IMotor.h                      # Motor interface
├── DCMotor.h                     # Real motor
├── DriveTrain.h                  # Drive control
├── Navigation.h                  # IMU + magnetometer ← NEW
├── TelemetryPacket.h             # Data structure for BLE/WebSocket
├── BatteryMonitor.h              # Battery ADC
├── ConfigManager.h               # NVS manager
├── BluetoothManager.h            # BLE
├── CommsManager.h                # WebSocket
└── CommandHandler.h              # Command parser

data/
├── index.html                    # Web dashboard
├── style.css                     # Styling
└── script.js                     # Interactivity
```

---

## 🧭 Magnetometer Calibration Workflow (Phase 1.5 - CURRENT PRIORITY)

### **Hard-Iron Calibration Process**

```
1. Robot at startup
   ↓
2. Calibrate Gyro Bias (200 samples, must be still)
   ↓
3. Start Magnetometer Calibration (via BLE command from robot_remote.py)
   ↓
4. Robot rotates 360° slowly (collect min/max magnetic field values)
   ↓
5. Calculate offsets: offset_x = (max_x + min_x) / 2, offset_y = (max_y + min_y) / 2
   ↓
6. Apply Hard-Iron correction to all future heading readings
   ↓
7. Use Complementary Filter (Gyro + Corrected Mag) for stable heading
   ↓
8. BLE streams calibration progress (0-100%) to robot_remote.py GUI in real-time
```

### **BLE Telemetry During Calibration**

robot_remote.py receives telemetry packets:
```json
{
  "heading": 45.2,           // Current heading (degrees)
  "calib_progress": 75,      // Calibration progress (0-100%)
  "is_calibrating": true,    // Calibration active
  "mag_x": 12.34,
  "mag_y": -8.56,
  "mag_z": 0.12
}
```

### **TelemetryPacket Structure**

```cpp
struct TelemetryPacket {
    // Robot pose (odometry + IMU fusion)
    float robotX, robotY, robotHeading;  // X, Y in mm, heading in degrees
    
    // Raw IMU data (for debugging/analysis)
    float accelX, accelY, accelZ;        // m/s²
    float gyroX, gyroY, gyroZ;           // rad/s
    float magX, magY, magZ;              // µT (raw or corrected)
    
    // Calibration state (streamed to BLE during setup)
    bool isCalibrated;
    int calibrationProgress;             // 0-100%
    float magOffsetX, magOffsetY;        // Hard-iron offsets
    
    // Motor feedback
    float leftWheelSpeed, rightWheelSpeed;  // ticks/s
    long leftWheelSteps, rightWheelSteps;   // encoder counts
    
    // Battery
    float batteryVoltage;                // Volts
};
```

---

## 🔨 Build Command Flow

```
platformio.ini
  └─ [env:esp32_real] → Hardware motors (DCMotor with PCNT encoder reading)

src/main.cpp
  1. Config initialization
  2. Motor setup (DCMotor + PCNT)
  3. BLE + WiFi startup
  4. Main loop:
     - Read PCNT encoders → update odometry
     - Process BLE commands
     - Send telemetry via WebSocket
```

---

## ✅ Success Criteria for Phase 1, 1.5 & 2

- [ ] **Build succeeds** without errors
- [ ] **Motor control**: Can set speed, motors respond ✅
- [ ] **Encoders work**: Position tracked in odometry ✅
- [ ] **Gyro calibration**: Startup bias removed, stable reading
- [ ] **Magnetometer calibration**: Hard-iron offsets computed via 360° rotation
- [ ] **Heading accuracy**: Gets true North after calibration
- [ ] **BLE connects**: robot_remote.py shows "Connected"
- [ ] **BLE telemetry**: Calibration progress (%) streams during setup sequence
- [ ] **WiFi telemetry**: Web dashboard receives X/Y/Heading updates
- [ ] **Battery monitoring**: ADC reading valid
- [ ] **Config persistence**: Settings saved in NVS

---

## 🚀 Next Actions

1. ✅ Create `Config.h` with all pins & constants
2. ✅ Implement `IMotor.h` interface
3. ✅ Implement `DCMotor` with encoder PCNT reading
4. ✅ Implement `DriveTrain` kinematics
5. ✅ **Create `Navigation.cpp`** - IMU fusion + magnetometer calibration
6. ✅ **Create `TelemetryPacket.h`** - Data structure for BLE streaming
7. ✅ **Create `main.cpp`** - Setup motors + IMU, calibration sequence, main loop
8. ✅ Copy communication layer from PREVIOUS/ (BLE, WiFi, telemetry)
9. 🔄 **BUILD & TEST** (next step)
10. 🔄 Verify motor control with hardware
11. 🔄 Test BLE connection with robot_remote.py
12. 🔄 Run magnetometer calibration sequence (rotate 360°)
13. 🔄 Verify heading accuracy

---

**Version**: 1.0 | **Date**: 2026-06-04 | **Owner**: Robot Team
