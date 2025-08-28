# Dual-MPU Kyphotic-Lordotic Posture Monitor

A real-time posture monitoring system using dual MPU6050 sensors to detect and correct kyphotic (upper back) and lordotic (lower back) postural deviations. The system provides immediate haptic feedback through servo vibrations and logs comprehensive analytics to ThingSpeak for long-term posture health tracking.

## 🎯 Features

### Core Functionality
- **Dual-sensor monitoring**: Separate tracking of upper back (kyphosis) and lower back (lordosis) 
- **Continuous haptic feedback**: Servo motors provide vibration alerts for bad posture
- **Intelligent motion detection**: Automatically switches between sitting and walking modes
- **One-button calibration**: Quick neutral posture setup with button press
- **Real-time analytics**: Tracks posture quality, incidents, and session duration

### Advanced Capabilities
- **Wi-Fi connectivity**: Automatic connection and data transmission
- **Cloud logging**: Comprehensive 8-field ThingSpeak integration
- **Quality scoring**: Real-time posture quality percentage calculation
- **Mode-aware tracking**: Separate statistics for sitting vs walking posture
- **Severity-based feedback**: Vibration intensity scales with posture deviation

## 🔧 Hardware Requirements

### Core Components
- **Arduino Uno/Nano** (or compatible)
- **2x MPU6050** IMU sensors (6-axis gyroscope + accelerometer)
- **2x Servo motors** (SG90 or similar)
- **ESP8266** Wi-Fi module
- **Push button** (normally open)
- **Breadboard and jumper wires**

### Optional Components
- **10kΩ resistor** (if button doesn't have internal pullup)
- **Power supply** (7-12V for servos)
- **3D printed enclosure** (for wearable mounting)

## 📡 Circuit Diagram

### Wiring Connections

```
Arduino Uno Connections:
┌─────────────────┬──────────────────┬─────────────────┐
│ Component       │ Arduino Pin      │ Notes           │
├─────────────────┼──────────────────┼─────────────────┤
│ Upper MPU6050   │ A4 (SDA), A5(SCL)│ Address: 0x68   │
│ Lower MPU6050   │ A4 (SDA), A5(SCL)│ Address: 0x69   │
│ Upper Servo     │ D3 (PWM)         │ Signal pin      │
│ Lower Servo     │ D5 (PWM)         │ Signal pin      │
│ Calibrate Button│ D2               │ Pull to GND     │
│ ESP8266 RX      │ D8               │ SoftwareSerial  │
│ ESP8266 TX      │ D9               │ SoftwareSerial  │
└─────────────────┴──────────────────┴─────────────────┘
```

### MPU6050 Sensor Placement
- **Upper MPU**: Mount with pins facing **toward feet** (positive X-axis down)
- **Lower MPU**: Mount with pins facing **toward head** (positive X-axis up)
- Both sensors share I2C bus but have different addresses

### Power Supply Notes
- MPU6050: 3.3V or 5V
- Servos: 5V (can draw high current)
- ESP8266: 3.3V (use voltage divider if needed)

## 📋 Installation

### 1. Library Dependencies
Install these libraries through Arduino IDE Library Manager:
```
- I2Cdev library (Jeff Rowberg)
- MPU6050 library (Jeff Rowberg) 
- Servo library (built-in)
- SoftwareSerial library (built-in)
```

### 2. Hardware Setup
1. Connect components according to wiring diagram
2. Mount upper MPU6050 on upper back/shoulders
3. Mount lower MPU6050 on lower back/lumbar region
4. Position servos for comfortable tactile feedback
5. Ensure button is easily accessible for calibration

### 3. Software Configuration
1. Clone this repository
2. Open `posture_monitor.ino` in Arduino IDE
3. Update Wi-Fi credentials:
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";       
   const char* WIFI_PASS = "YourPassword";
   ```
4. Configure ThingSpeak API key:
   ```cpp
   const char* TS_APIKEY = "YourThingSpeakAPIKey";
   ```
5. Upload to Arduino

## 🚀 Usage Guide

### Initial Setup
1. **Power on** the system
2. **Sit in perfect posture** (straight back, shoulders aligned)
3. **Press and hold** calibration button until calibration completes
4. System is now ready for monitoring

### Operation Modes
- **Sitting Mode**: Monitors desk/chair posture
- **Walking Mode**: Automatically detected via motion patterns
- **Calibration Mode**: Temporarily active during button press

### Feedback System
- **No vibration**: Good posture maintained
- **Upper servo vibrates**: Forward head/rounded shoulders detected
- **Lower servo vibrates**: Excessive lumbar curve (lordosis) detected
- **Both vibrate**: Both postural issues present simultaneously

### Recalibration
- Press calibration button anytime to reset neutral posture
- Recommended after position changes or clothing adjustments

## ⚙️ Configuration Options

### Sensitivity Tuning
```cpp
float DEAD_DEG = 0.8f;    // Threshold for bad posture (lower = more sensitive)
float GAIN = 3.5f;        // Vibration response strength
```

### Vibration Settings
```cpp
const unsigned long VIBE_PERIOD_MS = 250;  // Vibration speed
const int VIBE_AMPLITUDE = 25;             // Vibration strength
```

### Motion Detection
```cpp
const unsigned long MOTION_CHECK_MS = 300;  // Walking detection frequency
```

### Data Logging
```cpp
const unsigned long THINGSPEAK_INTERVAL_MS = 20000;  // Upload frequency
```

## 📊 ThingSpeak Data Fields

The system logs 8 comprehensive metrics:

| Field | Metric | Description |
|-------|---------|-------------|
| Field 1 | Upper Back Angle | Real-time upper back deviation (degrees) |
| Field 2 | Lordosis Angle | Real-time lower back curvature (degrees) |
| Field 3 | Sitting Bad Posture Count | Total incidents while sitting |
| Field 4 | Walking Bad Posture Count | Total incidents while walking |
| Field 5 | System Mode | 0=Sitting, 1=Walking, 2=Calibrating |
| Field 6 | Posture Quality Score | 0-100% session quality rating |
| Field 7 | Session Duration | Total monitoring time (minutes) |
| Field 8 | Current Status | 0=Good posture, 1=Bad posture |

## 🏥 Health & Safety

### Postural Conditions Monitored
- **Kyphosis**: Excessive forward curvature of upper spine
- **Lordosis**: Excessive inward curvature of lower spine
- **Forward head posture**: Head positioned ahead of shoulders

### Usage Recommendations
- Use as a training aid, not medical diagnosis tool
- Consult healthcare professionals for persistent postural issues
- Take regular breaks regardless of system feedback
- Gradually improve posture rather than forcing immediate corrections

## 🔧 Troubleshooting

### Common Issues

**Servos not vibrating:**
- Check power supply (servos need adequate current)
- Verify wiring connections
- Ensure calibration was completed successfully

**Incorrect vibration direction:**
- Adjust `UP_DIR` and `LOW_DIR` values (+1 or -1)
- Check MPU orientation matches code expectations

**Wi-Fi connection fails:**
- Verify SSID and password
- Check ESP8266 wiring and power
- System continues offline if Wi-Fi fails

**Calibration issues:**
- Ensure sensors are stable during calibration
- Verify both MPU6050s are detected on I2C bus
- Check I2C address configuration (0x68, 0x69)

### Debug Output
Monitor Serial output (115200 baud) for:
- Sensor initialization status
- Calibration values
- Real-time posture measurements
- Wi-Fi connection status
- ThingSpeak upload confirmations

## 🛠️ Customization

### Adjust Sensitivity
```cpp
// More sensitive (triggers easier)
float DEAD_DEG = 0.5f;

// Less sensitive (requires more deviation)
float DEAD_DEG = 1.5f;
```

### Modify Vibration Patterns
```cpp
// Faster vibration
const unsigned long VIBE_PERIOD_MS = 150;

// Stronger vibration
const int VIBE_AMPLITUDE = 35;
```

### Change MPU Orientations
If your sensor mounting differs:
```cpp
int UPPER_PITCH_SIGN = -1;  // Flip if forward/back feels inverted
int LOWER_PITCH_SIGN = +1;  // Flip if forward/back feels inverted
```

## 📈 Data Analytics

### Session Statistics
- Real-time posture quality scoring (0-100%)
- Separate incident tracking for sitting vs walking
- Average deviation measurements
- Severe posture deviation counting

### ThingSpeak Integration
Create a free ThingSpeak account to:
- View historical posture trends
- Set up custom alerts
- Generate posture improvement reports
- Share data with healthcare providers


**⚠️ Medical Disclaimer**: This device is for educational and training purposes only. It is not intended to diagnose, treat, cure, or prevent any medical condition. Consult healthcare professionals for persistent postural problems or pain.
