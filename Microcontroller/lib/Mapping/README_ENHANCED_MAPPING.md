# Enhanced VL53L0X Mapping System

## Visión General

Este sistema de mapeo mejorado proporciona capacidades avanzadas para aplicaciones de LiDAR VL53L0X montado en servomotor, incluyendo modo continuo, filtrado de datos, compensación por movimiento y muestreo adaptativo.

## Mejoras Implementadas

### ✅ **Modo Continuo del VL53L0X**
- **Problema**: El código original usaba modo single-shot con polling, limitando la frecuencia de muestreo
- **Solución**: Implementación de modo continuo con hasta 50Hz de frecuencia de medición
- **Beneficios**: Reducción de latencia y mayor throughput de datos

### ✅ **Configuración de Timing Budget**
- **Problema**: Sin optimización de velocidad vs precisión 
- **Solución**: Configuración flexible del timing budget (15ms-100ms)
- **Beneficios**: Balance optimizable entre velocidad y precisión según aplicación

### ✅ **Filtrado de Datos Avanzado**
- **Problema**: Datos ruidosos durante movimiento del servo
- **Solución**: Filtro de promedio móvil configurable + validación de calidad
- **Beneficios**: Datos más estables y confiables

### ✅ **Compensación por Movimiento**
- **Problema**: No considera movimiento del robot durante mapeo
- **Solución**: Compensación de coordenadas por velocidad del robot
- **Beneficios**: Mapeo preciso incluso con robot en movimiento

### ✅ **Muestreo Adaptativo**
- **Problema**: Frecuencia fija sin considerar velocidad angular del servo
- **Solución**: Ajuste automático de frecuencia según velocidad del servo
- **Beneficios**: Resolución angular óptima sin desperdiciar recursos

### ✅ **Sistema de Calibración**
- **Problema**: Offset hardcodeado (-38mm) sin calibración
- **Solución**: Sistema automático de calibración con objeto conocido
- **Beneficios**: Precisión mejorada y adaptable a diferentes configuraciones

### ✅ **Interpolación de Datos**
- **Problema**: Huecos en datos para ángulos específicos
- **Solución**: Interpolación lineal entre mediciones válidas
- **Beneficios**: Mapeo denso y completo de 360°

### ✅ **Métricas de Rendimiento**
- **Problema**: Sin visibilidad del rendimiento del sistema
- **Solución**: Estadísticas completas de mediciones y calidad
- **Beneficios**: Monitoreo y optimización del sistema

## Configuraciones Predefinidas

### MAPPING_CONFIG_DEFAULT
```c
.timing_budget_us = 20000,           // 20ms - balance velocidad/precisión
.measurement_period_ms = 25,         // 40Hz de frecuencia
.servo_angular_velocity = 30.0f,     // 30°/s velocidad servo
.filter_window_size = 3,             // Filtro suave
.motion_compensation_enabled = false, // Deshabilitado por defecto
.adaptive_sampling_enabled = true,   // Muestreo adaptativo
```

### MAPPING_CONFIG_HIGH_SPEED
```c
.timing_budget_us = 15000,           // 15ms - velocidad máxima
.measurement_period_ms = 20,         // 50Hz de frecuencia
.servo_angular_velocity = 60.0f,     // 60°/s velocidad alta
.filter_window_size = 2,             // Filtro mínimo
.motion_compensation_enabled = true, // Para robots móviles
```

### MAPPING_CONFIG_HIGH_ACCURACY
```c
.timing_budget_us = 50000,           // 50ms - máxima precisión
.measurement_period_ms = 60,         // 16Hz de frecuencia
.servo_angular_velocity = 15.0f,     // 15°/s velocidad lenta
.filter_window_size = 5,             // Filtro fuerte
.motion_compensation_enabled = true, // Compensación completa
```

## Uso Básico

### Inicialización
```c
#include "mapping.h"

// Usar configuración por defecto
esp_err_t err = mapping_init_enhanced(&MAPPING_CONFIG_DEFAULT);

// O configuración personalizada
mapping_config_t custom_config = {
    .timing_budget_us = 25000,
    .measurement_period_ms = 30,
    .servo_angular_velocity = 45.0f,
    // ... otros parámetros
};
err = mapping_init_enhanced(&custom_config);
```

### Mapeo con Callback
```c
void mi_callback(const mapping_measurement_t *measurement) {
    if (measurement->valid) {
        printf("Ángulo: %d°, Distancia: %dmm, Calidad: %d%%\n",
               measurement->angle_deg,
               measurement->distance_mm,
               measurement->quality_score);
    }
}

// Iniciar mapeo
mapping_start_enhanced(mi_callback);
```

### Lectura Manual
```c
mapping_measurement_t measurement;
esp_err_t err = mapping_get_measurement_enhanced(&measurement);

if (err == ESP_OK && measurement.valid) {
    // Procesar medición
    printf("Distancia: %dmm en ángulo %d°\n", 
           measurement.distance_mm, measurement.angle_deg);
}
```

### Compensación por Movimiento
```c
// Habilitar compensación
mapping_set_motion_compensation(true);

// Actualizar velocidad del robot periódicamente
mapping_set_robot_motion(
    velocity_x_mm_s,     // Velocidad X en mm/s
    velocity_y_mm_s,     // Velocidad Y en mm/s  
    angular_velocity_rad_s // Velocidad angular en rad/s
);
```

### Calibración
```c
// Colocar objeto a distancia conocida (ej: 500mm)
esp_err_t err = mapping_calibrate(500);
if (err == ESP_OK) {
    printf("Calibración exitosa\n");
}
```

### Interpolación
```c
// Obtener medición interpolada para ángulo específico
mapping_measurement_t measurement;
esp_err_t err = mapping_get_interpolated_measurement(45, &measurement);

if (err == ESP_OK) {
    printf("Distancia interpolada en 45°: %dmm\n", measurement.distance_mm);
}
```

### Estadísticas
```c
mapping_statistics_t stats;
esp_err_t err = mapping_get_statistics(&stats);

if (err == ESP_OK) {
    printf("Mediciones totales: %lu\n", stats.total_measurements);
    printf("Mediciones válidas: %lu\n", stats.valid_measurements);
    printf("Frecuencia: %.1f Hz\n", stats.measurement_rate_hz);
    printf("Fuerza señal promedio: %.1f\n", stats.average_signal_strength);
}
```

## Características Técnicas

### Rendimiento
- **Frecuencia máxima**: 50Hz en modo continuo
- **Rango de medición**: 30mm - 2000mm (configurable)
- **Precisión**: ±3% hasta 1.2m (VL53L0X estándar)
- **Resolución angular**: Dependiente de velocidad servo y frecuencia

### Filtrado
- **Tipo**: Promedio móvil configurable (1-10 muestras)
- **Validación**: Por estado de rango, fuerza señal y luz ambiente
- **Score de calidad**: 0-100% para cada medición

### Memoria
- **Buffer interpolación**: 360 mediciones (una por grado)
- **Buffer filtro**: Hasta 10 muestras por canal
- **Colas FreeRTOS**: 10 mediciones en cola
- **Stack tarea**: 4KB

## Compatibilidad

### Backward Compatibility
El sistema mantiene compatibilidad con el código existente:
```c
// Funciones legacy siguen funcionando
esp_err_t err = mapping_init();
err = getMappingValue(&angle, &distance);
```

### Hardware Requerido
- **VL53L0X**: Sensor principal
- **Servo de rotación continua**: Para mapeo 360°
- **GPIO interrupt**: Opcional, para máximo rendimiento
- **I2C**: Para comunicación con VL53L0X

## Optimizaciones Adicionales Recomendadas

### Para Aplicaciones de Alta Velocidad
1. **Usar GPIO interrupt** para sincronización precisa
2. **Configurar timing budget mínimo** (15ms)
3. **Reducir ventana de filtro** (1-2 muestras)
4. **Habilitar muestreo adaptativo**

### Para Máxima Precisión
1. **Usar timing budget alto** (50-100ms)  
2. **Aumentar ventana de filtro** (5-10 muestras)
3. **Calibrar sistema** con objetos conocidos
4. **Habilitar compensación por movimiento**

### Para Robots Móviles
1. **Habilitar compensación por movimiento**
2. **Actualizar velocidades** cada 50-100ms
3. **Usar configuración HIGH_SPEED**
4. **Monitorear estadísticas** de calidad

## Troubleshooting

### Problema: Baja frecuencia de medición
- **Verificar**: Timing budget muy alto
- **Solución**: Reducir a 15-20ms para aplicaciones rápidas

### Problema: Datos ruidosos
- **Verificar**: Ventana de filtro muy pequeña
- **Solución**: Aumentar filter_window_size a 3-5

### Problema: Mediciones inválidas
- **Verificar**: Límites de rango muy restrictivos
- **Solución**: Ajustar min_range_mm y max_range_mm

### Problema: Alto uso de CPU
- **Verificar**: Frecuencia muy alta sin interrupt
- **Solución**: Instalar interrupt handler o reducir frecuencia

## Ejemplos Completos

Ver `mapping_example.c` para ejemplos detallados de:
- Uso básico con configuraciones predefinidas
- Mapeo de alta velocidad con compensación
- Lectura manual con interpolación  
- Calibración del sistema
- Comparación de rendimiento

## Futuras Mejoras

### Pendientes de Implementación
- [ ] **Filtro Kalman**: Para mejor rechazo de ruido
- [ ] **Múltiples sensores**: Soporte para arrays de VL53L0X
- [ ] **Compresión de datos**: Para almacenamiento eficiente
- [ ] **Detección de obstáculos dinámicos**: Filtrado temporal
- [ ] **Mapeo SLAM**: Integración con odometría

### Optimizaciones de Hardware
- [ ] **PCB dedicado**: Para reducir ruido
- [ ] **Óptica mejorada**: Para mayor alcance
- [ ] **IMU integrada**: Para compensación de vibración 