# Servo Angle Reading for Continuous Rotation Servos

## Overview

This implementation provides angle reading functionality for continuous rotation servos by tracking their movement over time. Since continuous rotation servos don't have built-in position feedback, this solution estimates the angular position based on the servo's speed, direction, and elapsed time.

## Features

- **Real-time angle tracking**: Continuously monitors servo movement and calculates current angular position
- **Speed-based calculation**: Uses servo speed and direction to estimate angular velocity
- **Normalized angles**: Returns angles in the range 0-359 degrees
- **Thread-safe**: Uses semaphores to ensure thread safety
- **Reset functionality**: Allows resetting the angle counter to 0 degrees
- **Compatibility**: Works with existing servo compatibility layer

## How It Works

The angle tracking system works by:

1. **Monitoring servo state**: Tracks the current speed, direction, and timing
2. **Calculating angular velocity**: Estimates rotation speed based on servo speed settings
3. **Time-based integration**: Accumulates angle changes over time
4. **Normalization**: Keeps angles within 0-359 degree range

### Mathematical Model

```
angular_velocity = speed_factor × direction_factor × base_speed
angle_change = angular_velocity × elapsed_time
current_angle += angle_change
```

Where:
- `speed_factor` = servo speed (0-100) / 100
- `direction_factor` = +1 for CCW, -1 for CW
- `base_speed` = approximately 6 degrees/second per speed unit
- `elapsed_time` = time since last update in seconds

## API Functions

### `int16_t readAngle_simple(void)`
Returns the current tracked angle in degrees (0-359).

**Returns:**
- Current angle in degrees (0-359)
- -1 if servo is not initialized

**Example:**
```c
int16_t current_angle = readAngle_simple();
printf("Current angle: %d degrees\n", current_angle);
```

### `void servo_simple_reset_angle(void)`
Resets the angle counter to 0 degrees.

**Example:**
```c
servo_simple_reset_angle();
printf("Angle reset to 0 degrees\n");
```

## Usage Example

```c
#include "servo_compatibility.h"
#include "esp_log.h"

void servo_angle_example(void)
{
    // Initialize and start servo
    servo_simple_initialize();
    servo_simple_start();
    
    // Reset angle to 0
    servo_simple_reset_angle();
    
    // Read angle during movement
    for (int i = 0; i < 10; i++) {
        int16_t angle = readAngle_simple();
        ESP_LOGI("SERVO", "Current angle: %d degrees", angle);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Stop servo and read final angle
    servo_simple_stop();
    int16_t final_angle = readAngle_simple();
    ESP_LOGI("SERVO", "Final angle: %d degrees", final_angle);
}
```

## Accuracy Considerations

The angle tracking accuracy depends on several factors:

1. **Servo speed consistency**: Real servo speed may vary from commanded speed
2. **Load variations**: Different loads affect actual rotation speed
3. **Timing precision**: System timing affects integration accuracy
4. **Calibration**: The base speed constant (6°/s) may need adjustment for specific servos

### Improving Accuracy

1. **Calibrate base speed**: Measure actual rotation speed and adjust the constant
2. **Add feedback**: Use external sensors (encoders, potentiometers) for verification
3. **Periodic reset**: Reset angle counter at known positions
4. **Filtering**: Apply smoothing algorithms to reduce noise

## Limitations

- **Estimated position**: Not as accurate as position servos with built-in feedback
- **Drift over time**: Accumulated errors can cause position drift
- **No absolute reference**: Requires manual reset to establish reference point
- **Speed dependency**: Accuracy depends on consistent servo speed

## Integration

The angle reading functionality is automatically integrated into the existing servo compatibility layer. All servo control functions (`servo_simple_start`, `servo_simple_stop`, `servo_simple_set_speed`, etc.) automatically update the angle tracking.

## Files Modified

- `servo_compatibility.c`: Added angle tracking implementation
- `servo_compatibility.h`: Added function declarations
- `servo_angle_example.c`: Example usage demonstration
- `servo_angle_example.h`: Example header file

## Dependencies

- ESP-IDF FreeRTOS
- ESP Timer API
- Generic servo library
- ESP Logging system