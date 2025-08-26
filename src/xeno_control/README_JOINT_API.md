# Joint Classes API Documentation

## Overview

The joint classes in xeno_control now provide getter and setter methods for reading/modifying both current motor values and calibration values through the OneMotor library.

## Joint Mapping

- **Joint 1**: Lift (uses dual M3508 motors)
- **Joint 2**: Stretch (uses dual M3508 motors)  
- **Joint 3**: Shift (uses single M3508 motor)
- **Joint 4-6**: Arm[1-3] (uses J4310 motors with IDs 1-3)
- **Joint 7**: Suck (uses single M3508 motor)

## New API Methods

### For All Joint Classes

Each joint class now provides these methods:

```cpp
// Read current motor position/velocity values
[[nodiscard]] tl::expected<float, OneMotor::Error> getPosition() const;
[[nodiscard]] tl::expected<float, OneMotor::Error> getVelocity() const;

// Read calibration values
[[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationPosition() const;
[[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationVelocity() const;

// Modify calibration values
[[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationPosition(float position);
[[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationVelocity(float velocity);
```

### Arm Class Special Case

The Arm class handles multiple motors, so it requires a motor ID parameter:

```cpp
// Read current motor position/velocity values
[[nodiscard]] tl::expected<float, OneMotor::Error> getPosition(uint8_t id) const;
[[nodiscard]] tl::expected<float, OneMotor::Error> getVelocity(uint8_t id) const;

// Read calibration values
[[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationPosition(uint8_t id) const;
[[nodiscard]] tl::expected<float, OneMotor::Error> getCalibrationVelocity(uint8_t id) const;

// Modify calibration values
[[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationPosition(uint8_t id, float position);
[[nodiscard]] tl::expected<void, OneMotor::Error> setCalibrationVelocity(uint8_t id, float velocity);
```

Where `id` should be 1, 2, or 3 for the three arm joints.

## Hardware Interface Integration

The `XenoHardware` class now provides public methods for accessing calibration values:

```cpp
// Get calibration values
bool getCalibrationPosition(int joint_id, double& position) const;
bool getCalibrationVelocity(int joint_id, double& velocity) const;

// Set calibration values  
bool setCalibrationPosition(int joint_id, double position);
bool setCalibrationVelocity(int joint_id, double velocity);
```

## Implementation Details

- All methods use `tl::expected<T, OneMotor::Error>` for error handling
- The hardware interface automatically maps joint IDs (1-7) to appropriate joint classes
- For dual motor joints (Lift, Stretch), calibration changes apply to both motors with appropriate sign handling
- Real hardware mode (non-simulation) now reads actual motor values and sends commands through OneMotor

## Error Handling

All methods return `tl::expected` types. Check for errors before using values:

```cpp
auto position = Lift::getInstance().getPosition();
if (position.has_value()) {
    float pos_value = position.value();
    // Use position value
} else {
    // Handle error: position.error()
}
```