# Mobility Management

## List of Component
- **DC Motor Model:** JGA25-370 [Link](https://www.amazon.sg/JGA25-370-Geared-Electric-Torque-1360rpm/dp/B0BCFS43VW?th=1)
   - **Voltage**: 12V
   - **Speed**: 280 RPM
   - **Stall Torque**: 1.7 KG.CM
   - **Encoder**: JGA25-370
   - Why chose this? Because it fits perfectly for robot:
     - Precision & Control: It's ideal for steering, also offer nuanced directional control for the vehicle
     - Balance both power & speed: Provide enough torque to move the car while maintaining stable speed.
     - Built-in Encoders: the built-in encoder act as an eye for the motor, feeding real-time data back to the board.
 - **Base Chassis** : Lower Chassis [Part](https://youtu.be/ulTM1uV1Bvg?si=f2Lna6793y4V5BLy)
 - **Upper Chassis** : [3D Printing Part]() that use to store Jetson Nano Orin
 - **Servo** :
 - **Wheel** : Part [Link](https://youtu.be/ulTM1uV1Bvg?si=f2Lna6793y4V5BLy)
 - **Hokuyo UST-30LX Lidar** : [Link](https://www.hokuyo-aut.jp/search/single.php?serial=233)
   - **Max Range** : 30 meters
   - **Detection Angle** : 270 degrees
   - **Anti-ambient light**
 - **Intel RealSense Depth Camera D455** : [Link](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html)
 - **Custom Motor Controller Board** 
 - **Control Area Network(CAN) Bus**
 - **RealSense Camera Mounting** : [3D Printed Part]()
 - **Lidar Mounting** : [3D Printed Part]()
 - **IMU BNO055** : Mainly use For lap counting [Link](https://www.adafruit.com/product/2472?srsltid=AfmBOor3sAo9Rdx37jv7NFbS13hn4i-PcHyRNyb_V6hFcZzWmqiRIFs9)
 - **Other Parts**

## Power
### Battery Detail
TCB Li-Po Rechargeable Battery (3S 1100mAh)

- Brand: TCB
Battery Cell Composition: Lithium Polymer (Li-Po)
- Battery Capacity: 1100 Milliamp Hours
- Capacity: 1100 mAh. 
- Estimated recharge time: 1.5 – 2 hours.

Power Supply Hierarchy:
- 2 Battery 12 V each
- One for:
  -  for Jetson 12 V (RealSense Camera and other through USB)
- Another for: 
  - Lidar 12 V
  - Motor 12 V

##  Custom Motor Controller Board Documentation

### Overview

Our coach help designed and manufactured a custom PCB (Printed Circuit Board) to serve as the central control system for our robot. This board integrates motor control, CAN bus communication, power distribution, and sensor interfaces into a single, compact solution.

### Board Images

**Image 1(Board.jpeg) : Top Layer (Component Side)**
![3D_Board](./Board_3D.jpeg)
- Green PCB with clearly labeled sections
- White silkscreen labels for easy identification
- Organized layout with dedicated functional zones

**Image 2: Bottom Layer (Copper Traces)** - Reveals the internal wiring and circuit routing
![Board](./Board.jpeg)
- Red copper traces show signal and power routing
- Professional PCB design with proper trace spacing
- Ground planes visible for noise reduction

**Image 3: Schematic Diagram** - Complete electrical design documentation
![Schematic](./Schematic.jpeg)
- Full circuit schematic showing all connections
- Component values and part numbers specified
- Functional blocks clearly defined

---

### Board Architecture & Key Sections

#### 1. **Microcontroller Section**
- **MCU**: STM32F103C8Tx (top-left section)
- **Function**: Main processing unit for motor control and CAN communication
- **Features**: 
  - 72MHz ARM Cortex-M3 processor
  - Hardware CAN bus controller
  - Multiple PWM outputs for motor control
  - GPIO pins for sensor interfaces

#### 2. **CAN Bus Interface** (Top-right section labeled "CANBUS")
- **Transceiver**: TJA1050 CAN transceiver chip
- **Connectors**: 
  - CANH and CANL terminals for differential signaling
  - 120Ω termination resistor visible in schematic
- **Purpose**: Reliable communication between Jetson Orin Nano and motor controller
- **Data Rate**: Supports up to 1 Mbps for real-time control

#### 3. **Motor Driver Section** (Bottom-right)
- **Motor Connectors**: 
  - Motor1 header (6-pin) - Drive motor connection
  - Motor2 header (6-pin) - Servo/second motor connection
- **Driver Chips**: TB6612FNG dual motor driver (visible in schematic)
- **Power Handling**: 
  - Supports up to 12V input
  - 1.2A continuous per channel (3.2A peak)
- **Features**:
  - Bidirectional motor control (forward/reverse)
  - PWM speed control
  - Built-in protection (over-current, thermal shutdown)

#### 4. **Power Management** (Left side labeled "POWER")
- **Input**: 12V from main battery via barrel jack or screw terminals
- **Voltage Regulators**: 
  - 5V regulator for logic circuits (AMS1117-5.0)
  - 3.3V regulator for MCU and sensors (AMS1117-3.3)
- **Power Distribution**:
  - Separate power planes for motors and logic
  - Decoupling capacitors (C1-C7) for stable power
  - Yellow LEDs indicate power status

#### 5. **Debugging & Programming Interface** (Top-left)
- **Debugger Port**: 4-pin SWD (Serial Wire Debug) connector
  - Allows firmware programming via ST-Link
  - Real-time debugging capability
- **Purpose**: Development and troubleshooting during testing

#### 6. **GPIO & Peripheral Connections** (Left side)
- **UART Port**: For serial communication with Jetson
- **PB8-PB15 Pins**: General-purpose I/O for sensors
- **PCL1_LED**: Status LED output
- **Servo POWER**: Dedicated servo power output

#### 7. **Boot & Reset Circuit** (Top-center in schematic)
- **Boot Selection**: Jumper-configurable boot mode
  - Boot from flash memory (normal operation)
  - Boot from bootloader (firmware update)
- **Reset Button**: Manual reset capability
- **Status LEDs**: 
  - D1 (LED) - Power indicator
  - D5 (1k resistor) - Activity indicator

#### 8. **Oscillator Circuit** (Center in schematic)
- **Crystal**: 8MHz external crystal (Crystal_GND24)
- **Purpose**: Provides stable clock signal for MCU
- **Capacitors**: C2 and C8 for crystal stabilization

---

### Circuit Design Highlights

#### Power Distribution Network
```
12V Battery Input
    ↓
[LM2596-5 Buck Converter] → 5V Rail
    ↓                           ↓
[AMS1117-3.3]            Motor Drivers
    ↓                      (up to 1.2A each)
3.3V Rail
    ↓
STM32 MCU + Peripherals
```

#### Signal Flow
```
Jetson Orin Nano
    ↓ (via CAN bus)
TJA1050 Transceiver
    ↓
STM32F103 MCU
    ↓ (PWM signals)
TB6612FNG Motor Drivers
    ↓
DC Motors (Drive + Servo)
```

---

### Technical Specifications

| Parameter | Specification |
|-----------|--------------|
| **Board Dimensions** | ~80mm × 80mm (approx.) |
| **PCB Layers** | 2-layer FR4 |
| **Input Voltage** | 12V DC (9-16V range) |
| **Logic Voltage** | 3.3V (MCU), 5V (peripherals) |
| **Motor Channels** | 2 independent channels |
| **Motor Current** | 1.2A continuous, 3.2A peak per channel |
| **CAN Bus Speed** | Up to 1 Mbps |
| **MCU Clock** | 72 MHz (ARM Cortex-M3) |
| **Programming Interface** | SWD (Serial Wire Debug) |

---

### Key Design Features

1. **Compact Integration**: All motor control, power, and communication in one board
2. **Professional Layout**: Organized functional sections with clear labeling
3. **Proper Grounding**: Ground planes reduce electrical noise
4. **Protection Circuits**: Over-current and thermal protection for motors
5. **Debugging Support**: Built-in programmer/debugger interface
6. **Flexible Configuration**: Boot jumpers and GPIO breakouts for customization
7. **Visual Feedback**: Status LEDs for power and activity monitoring

---

### Manufacturing & Assembly

- **PCB Fabrication**: Professionally manufactured (likely JLCPCB or similar)
- **Assembly Method**: Hand-soldered surface-mount and through-hole components
- **Quality**: Clean solder joints, proper component placement
- **Testing**: Functional testing performed before vehicle integration

---

### Connection to Main System

This custom board serves as the **motor control interface** between:
- **Input**: Jetson Orin Nano (high-level commands via CAN bus)
- **Output**: DC motors and servo (low-level PWM control)


---

### Board Files Available

- **Schematic**: Complete circuit diagram (shown in Image 3)
- **PCB Layout**: Top and bottom copper layers (Images 1 & 2)
- **Bill of Materials (BOM)**: Component list with part numbers
- **Gerber Files**: Manufacturing files for PCB production

*For complete technical details, refer to the schematic diagram and PCB layout files in this directory.*

---

