# Enhanced VL53L0X LiDAR Mapping Library

## 📋 Overview

This enhanced library transforms the basic VL53L0X sensor into a professional-grade mapping system suitable for real-time 360° mapping on moving robots with rotating servos. The library provides advanced features including continuous mode operation, data filtering, motion compensation, and adaptive sampling.

## 🚀 Key Improvements

### **Performance Enhancements**
- **Continuous Mode**: Up to 50Hz measurement rate (vs 20Hz polling)
- **Interrupt-Driven**: Non-blocking data acquisition
- **Adaptive Sampling**: Optimizes measurement rate based on servo speed
- **Motion Compensation**: Corrects measurements for robot movement

### **Data Quality**
- **Moving Average Filtering**: Configurable noise reduction (1-10 point window)
- **Signal Quality Assessment**: Automatic measurement validation
- **Calibration System**: Automatic offset correction
- **Interpolation**: Fills gaps in angular data

### **Advanced Features**
- **Multiple Configuration Presets**: Default, High-Speed, High-Accuracy
- **Real-time Statistics**: Performance monitoring and debugging
- **Backward Compatibility**: Works with existing code
- **Comprehensive Documentation**: Detailed parameter explanations

## 📁 File Structure

```
lib/Mapping/LiDAR_Library/
├── vl53l0x.h              # Enhanced VL53L0X sensor interface
├── vl53l0x.c              # Sensor implementation with continuous mode
├── mapping.h              # Advanced mapping system interface
├── mapping.c              # Mapping implementation with filtering & compensation
├── mapping_example.c      # Comprehensive usage examples
└── README_ENHANCED_MAPPING.md  # This documentation
```

## ⚙️ Configuration Parameters

### **Critical Parameters**

#### `servo_angular_velocity` (CRITICAL)
```c
float servo_angular_velocity = 30.0f;  // degrees per second
```
**MEASURE THIS**: Use a stopwatch to time one full rotation
- **5-15 deg/s**: Very slow, high precision mapping
- **15-30 deg/s**: Normal mapping speed  
- **30-60 deg/s**: Fast mapping
- **60-180 deg/s**: Very fast mapping (may miss objects)

**Example**: If servo takes 12 seconds for 180°, use `15.0f`

#### `timing_budget_us` (Speed vs Accuracy)
```c
uint32_t timing_budget_us = 20000;  // microseconds
```
- **15000 us (15ms)**: High speed, lower accuracy (~50Hz)
- **20000 us (20ms)**: Balanced speed/accuracy (~40Hz) ⭐ **Recommended**
- **33000 us (33ms)**: Better accuracy (~25Hz)
- **50000 us (50ms)**: High accuracy (~15Hz)
- **100000 us (100ms)**: Maximum accuracy (~8Hz)

#### `measurement_period_ms` (Measurement Rate)
```c
uint32_t measurement_period_ms = 25;  // milliseconds
```
- Must be >= `timing_budget_us/1000`
- Lower values = higher measurement rate
- Higher values = lower power consumption
- **Recommendation**: Use 25ms for 40Hz operation

### **Quality Control Parameters**

#### `filter_window_size` (Noise Reduction)
```c
uint8_t filter_window_size = 3;  // 1-10
```
- **1**: No filtering (raw data)
- **2-3**: Light filtering, fast response ⭐ **Recommended**
- **4-6**: Medium filtering, balanced
- **7-10**: Heavy filtering, slow response but stable

#### `signal_rate_limit` (Signal Quality)
```c
float signal_rate_limit = 0.1f;  // 0.05-0.5
```
- **0.05-0.1**: Accept weak signals (more measurements, lower quality)
- **0.1-0.2**: Balanced signal quality ⭐ **Recommended**
- **0.2-0.5**: High signal quality (fewer measurements, higher quality)

#### `min_range_mm` / `max_range_mm` (Valid Range)
```c
uint16_t min_range_mm = 50;   // 30-200 mm
uint16_t max_range_mm = 2000; // 1000-4000 mm
```
- **min_range_mm**: Objects below this distance are invalid
- **max_range_mm**: Objects above this distance are invalid
- **Recommendation**: 50-2000mm for most applications

### **Advanced Features**

#### `motion_compensation_enabled` (Robot Movement)
```c
bool motion_compensation_enabled = true;  // Enable for moving robots
```
- **true**: Corrects measurements based on robot velocity
- **false**: Use when robot is stationary
- Requires calling `mapping_set_robot_motion()` with current velocities

#### `adaptive_sampling_enabled` (Variable Speed Servos)
```c
bool adaptive_sampling_enabled = true;  // Optimize angular resolution
```
- **true**: Adjusts measurement rate based on servo speed
- **false**: Fixed measurement rate regardless of servo speed

## 🎯 Configuration Presets

### **Default Configuration** (Balanced)
```c
const mapping_config_t MAPPING_CONFIG_DEFAULT = {
    .timing_budget_us = 20000,          // 20ms (~40Hz)
    .measurement_period_ms = 25,        // 25ms between measurements
    .servo_angular_velocity = 30.0f,    // 30 deg/s (6s for 180°)
    .min_range_mm = 50,                 // 50mm minimum
    .max_range_mm = 2000,               // 2000mm maximum
    .filter_window_size = 3,            // 3-point filter
    .motion_compensation_enabled = false, // Disabled by default
    .adaptive_sampling_enabled = true,  // Adaptive sampling
    .signal_rate_limit = 0.1f           // 10% signal threshold
};
```
**Use for**: General indoor mapping, robot navigation, object detection

### **High-Speed Configuration** (Real-time)
```c
const mapping_config_t MAPPING_CONFIG_HIGH_SPEED = {
    .timing_budget_us = 15000,          // 15ms (~50Hz)
    .measurement_period_ms = 20,        // 20ms between measurements
    .servo_angular_velocity = 60.0f,    // 60 deg/s (3s for 180°)
    .min_range_mm = 50,                 // 50mm minimum
    .max_range_mm = 1500,               // 1500mm maximum (reduced for speed)
    .filter_window_size = 2,            // 2-point filter (minimal)
    .motion_compensation_enabled = true, // Motion compensation enabled
    .adaptive_sampling_enabled = true,  // Adaptive sampling
    .signal_rate_limit = 0.05f          // 5% signal threshold (permissive)
};
```
**Use for**: Real-time obstacle avoidance, fast-moving robots, dynamic environments

### **High-Accuracy Configuration** (Precision)
```c
const mapping_config_t MAPPING_CONFIG_HIGH_ACCURACY = {
    .timing_budget_us = 50000,          // 50ms (~15Hz)
    .measurement_period_ms = 60,        // 60ms between measurements
    .servo_angular_velocity = 15.0f,    // 15 deg/s (12s for 180°)
    .min_range_mm = 30,                 // 30mm minimum (very close)
    .max_range_mm = 2000,               // 2000mm maximum
    .filter_window_size = 5,            // 5-point filter (heavy filtering)
    .motion_compensation_enabled = true, // Motion compensation enabled
    .adaptive_sampling_enabled = false, // Fixed sampling for consistency
    .signal_rate_limit = 0.2f           // 20% signal threshold (high quality)
};
```
**Use for**: Detailed environmental mapping, precision measurements, calibration

## 💻 Usage Examples

### **Basic Usage**
```c
#include "Mapping/LiDAR_Library/mapping.h"

void basic_mapping_example(void) {
    // Initialize with default configuration
    esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_DEFAULT);
    if (err != ESP_OK) {
        printf("Failed to initialize mapping\n");
        return;
    }
    
    // Start mapping
    err = mapping_start_enhanced(NULL);
    if (err != ESP_OK) {
        printf("Failed to start mapping\n");
        return;
    }
    
    // Read measurements
    mapping_measurement_t measurement;
    while (1) {
        err = mapping_get_measurement_enhanced(&measurement);
        if (err == ESP_OK && measurement.valid) {
            printf("Angle: %d°, Distance: %dmm, Quality: %d%%\n",
                   measurement.angle_deg,
                   measurement.distance_mm,
                   measurement.quality_score);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### **High-Speed with Motion Compensation**
```c
void high_speed_mapping_example(void) {
    // Initialize with high-speed configuration
    esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_HIGH_SPEED);
    if (err != ESP_OK) return;
    
    // Enable motion compensation
    mapping_set_motion_compensation(true);
    
    // Set robot motion (moving forward at 100mm/s)
    mapping_set_robot_motion(100.0f, 0.0f, 0.0f);
    
    // Start mapping with callback
    err = mapping_start_enhanced(mapping_data_callback);
    if (err != ESP_OK) return;
    
    // Update robot motion periodically
    for (int i = 0; i < 20; i++) {
        float velocity_x = 100.0f * cos(i * 0.1f);
        float velocity_y = 50.0f * sin(i * 0.2f);
        float angular_vel = 0.1f * sin(i * 0.2f);
        
        mapping_set_robot_motion(velocity_x, velocity_y, angular_vel);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Callback function for new data
static void mapping_data_callback(const mapping_measurement_t *measurement) {
    if (measurement->valid) {
        printf("New measurement: Angle=%d°, Distance=%dmm, Quality=%d%%\n",
               measurement->angle_deg,
               measurement->distance_mm,
               measurement->quality_score);
    }
}
```

### **Calibration Example**
```c
void calibration_example(void) {
    // Create custom configuration for calibration
    mapping_config_t calib_config = {
        .timing_budget_us = 30000,          // 30ms for high accuracy
        .measurement_period_ms = 40,        // 40ms period
        .servo_angular_velocity = 20.0f,    // Slow servo for precision
        .min_range_mm = 30,                 // Very close objects
        .max_range_mm = 3000,               // Extended range
        .filter_window_size = 5,            // Heavy filtering
        .motion_compensation_enabled = true,
        .adaptive_sampling_enabled = false, // Fixed sampling
        .signal_rate_limit = 0.15f          // Higher signal threshold
    };
    
    // Initialize and start mapping
    esp_err_t err = mapping_init_enhanced(&calib_config);
    if (err != ESP_OK) return;
    
    mapping_start_enhanced(NULL);
    
    // Place known object at 500mm and calibrate
    printf("Place object at exactly 500mm from sensor\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    err = mapping_calibrate(500);  // Known distance of 500mm
    if (err == ESP_OK) {
        printf("Calibration successful!\n");
    }
    
    // Test calibrated measurements
    for (int i = 0; i < 10; i++) {
mapping_measurement_t measurement;
        err = mapping_get_measurement_enhanced(&measurement);
if (err == ESP_OK && measurement.valid) {
            printf("Raw: %dmm, Calibrated: %dmm, Quality: %d%%\n",
                   measurement.raw_distance_mm,
                   measurement.distance_mm,
                   measurement.quality_score);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## 📊 Performance Comparison

| Feature | Legacy Library | Enhanced Library | Improvement |
|---------|---------------|------------------|-------------|
| **Measurement Rate** | ~20Hz | ~40-50Hz | **2.5x faster** |
| **Data Quality** | Raw data only | Filtered + validated | **Significantly better** |
| **Motion Compensation** | None | Full 3D compensation | **New feature** |
| **Calibration** | Manual offset | Automatic system | **New feature** |
| **Interpolation** | None | Linear interpolation | **New feature** |
| **Statistics** | None | Real-time monitoring | **New feature** |
| **Configuration** | Hardcoded | Flexible presets | **Much more flexible** |

## 🔧 Troubleshooting

### **Common Issues**

#### **"Invalid timing budget" Error**
- Ensure `timing_budget_us` is between 15000-100000 microseconds
- Check that `measurement_period_ms` >= `timing_budget_us/1000`

#### **Poor Measurement Quality**
- Increase `signal_rate_limit` (try 0.2f)
- Increase `filter_window_size` (try 5)
- Check sensor alignment and environment lighting

#### **Missing Measurements**
- Decrease `signal_rate_limit` (try 0.05f)
- Check servo speed vs measurement rate
- Verify sensor is not too close to objects

#### **Incorrect Angular Data**
- **CRITICAL**: Measure and set correct `servo_angular_velocity`
- Use stopwatch to time one full rotation
- Example: 12 seconds for 180° = 15.0 deg/s

### **Performance Optimization**

#### **For Maximum Speed**
```c
mapping_config_t speed_config = {
    .timing_budget_us = 15000,          // 15ms
    .measurement_period_ms = 20,        // 20ms
    .servo_angular_velocity = 60.0f,    // Fast servo
    .filter_window_size = 1,            // No filtering
    .signal_rate_limit = 0.05f          // Accept weak signals
};
```

#### **For Maximum Accuracy**
```c
mapping_config_t accuracy_config = {
    .timing_budget_us = 100000,         // 100ms
    .measurement_period_ms = 120,       // 120ms
    .servo_angular_velocity = 10.0f,    // Slow servo
    .filter_window_size = 10,           // Heavy filtering
    .signal_rate_limit = 0.3f           // High quality only
};
```

## 🔮 Future Enhancements

### **Planned Features**
- **Multi-sensor support**: Multiple VL53L0X sensors
- **Advanced filtering**: Kalman filter implementation
- **SLAM integration**: Simultaneous Localization and Mapping
- **Cloud mapping**: Remote data processing
- **Machine learning**: Adaptive parameter optimization

### **Performance Targets**
- **100Hz measurement rate** with optimized timing
- **Sub-millimeter accuracy** with advanced calibration
- **Real-time 3D mapping** with multiple sensors
- **Edge AI integration** for object recognition

## 📚 API Reference

### **Core Functions**
```c
// Initialize enhanced mapping system
esp_err_t mapping_init_enhanced(const mapping_config_t *config);

// Start continuous mapping
esp_err_t mapping_start_enhanced(mapping_data_callback_t callback);

// Get enhanced measurement
esp_err_t mapping_get_measurement_enhanced(mapping_measurement_t *measurement);

// Configure robot motion for compensation
esp_err_t mapping_set_robot_motion(float velocity_x, float velocity_y, float angular_velocity);

// Calibrate system with known distance
esp_err_t mapping_calibrate(uint16_t known_distance_mm);

// Get interpolated measurement at specific angle
esp_err_t mapping_get_interpolated_measurement(int16_t target_angle, mapping_measurement_t *measurement);

// Get system statistics
esp_err_t mapping_get_statistics(mapping_statistics_t *stats);
```

### **Data Structures**
```c
typedef struct {
    uint16_t distance_mm;              // Filtered distance
    int16_t angle_deg;                 // Servo angle
    uint16_t raw_distance_mm;          // Raw unfiltered distance
    uint16_t signal_strength;          // Signal strength
    uint32_t timestamp_us;             // Timestamp
    float angular_velocity;            // Current servo velocity
    bool valid;                        // Measurement validity
    uint8_t quality_score;             // Quality score (0-100)
} mapping_measurement_t;

typedef struct {
    uint32_t total_measurements;       // Total measurements
    uint32_t valid_measurements;       // Valid measurements
    uint32_t filtered_measurements;    // Filtered out
    uint32_t interpolated_points;      // Interpolated points
    float average_signal_strength;     // Average signal strength
    float measurement_rate_hz;         // Current rate
    uint32_t error_count;              // Error count
} mapping_statistics_t;
```

## 🤝 Contributing

This enhanced library is designed to be easily extensible. Key areas for contribution:

1. **New filtering algorithms**
2. **Advanced motion compensation models**
3. **Multi-sensor synchronization**
4. **Performance optimizations**
5. **Additional configuration presets**

## 📄 License

This enhanced library maintains compatibility with the original VL53L0X library while adding significant new functionality. Please refer to the original library's license terms.

---

**Version**: 2.0  
**Date**: 2025-02-09  
**Author**: Enhanced for servo-mounted LiDAR mapping applications 