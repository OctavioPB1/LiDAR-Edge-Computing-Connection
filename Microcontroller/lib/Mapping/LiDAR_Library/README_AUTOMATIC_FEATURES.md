# Funcionalidades Automáticas del Sistema de Mapeo LiDAR

## Resumen

El sistema de mapeo LiDAR ahora incluye funcionalidades automáticas que se activan sin necesidad de modificar el código principal (`main.c`). Simplemente llamando a las funciones legacy (`mapping_init()`, `getMappingValue()`), el sistema automáticamente utiliza todas las mejoras avanzadas.

## Características Automáticas

### 1. **Inicialización Inteligente**
- **Función**: `mapping_init()`
- **Comportamiento**: Automáticamente detecta el entorno y configura los parámetros óptimos
- **Características activadas**:
  - Modo continuo del sensor VL53L0X (hasta 50Hz)
  - Filtrado de media móvil para estabilidad
  - Evaluación de calidad de mediciones
  - Interpolación de datos
  - Auto-calibración del sistema

### 2. **Medición Mejorada**
- **Función**: `getMappingValue(int16_t *angle, uint16_t *distance)`
- **Comportamiento**: Retorna mediciones con todas las mejoras aplicadas
- **Características incluidas**:
  - Filtrado automático de ruido
  - Compensación de movimiento (si está habilitada)
  - Calibración automática aplicada
  - Fallback a interpolación si falla la medición
  - Logging de calidad (configurable)

## Configuración

### Archivo de Configuración: `mapping_config.h`

Puedes personalizar el comportamiento sin modificar el código principal editando las siguientes opciones en `mapping_config.h`:

#### Funcionalidades Automáticas
```c
// Habilitar análisis automático del entorno
#define MAPPING_AUTO_ENVIRONMENT_ANALYSIS 1

// Habilitar auto-calibración
#define MAPPING_AUTO_CALIBRATION 1

// Habilitar interpolación automática como fallback
#define MAPPING_AUTO_INTERPOLATION 1

// Habilitar logging de calidad
#define MAPPING_QUALITY_LOGGING 1

// Habilitar compensación de movimiento por defecto
#define MAPPING_DEFAULT_MOTION_COMPENSATION 0
```

#### Ajustes de Rendimiento
```c
// Presupuesto de tiempo para análisis de entorno
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_50MS

// Número de muestras para análisis (10-50)
#define MAPPING_ANALYSIS_SAMPLES 20

// Número de muestras para calibración (5-20)
#define MAPPING_CALIBRATION_SAMPLES 10

// Calidad mínima para calibración (0-100)
#define MAPPING_CALIBRATION_MIN_QUALITY 50
```

#### Configuración de Logging
```c
// Intervalo de logging de calidad (microsegundos)
#define MAPPING_QUALITY_LOG_INTERVAL_US 5000000

// Logging detallado de inicialización
#define MAPPING_DETAILED_INIT_LOGGING 1
```

## Uso Básico

### Código Mínimo Requerido
```c
#include "mapping.h"

void app_main(void) {
    // Inicialización automática con todas las mejoras
    esp_err_t err = mapping_init();
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize mapping");
        return;
    }
    
    // Bucle principal
    while (1) {
        int16_t angle;
        uint16_t distance;
        
        // Obtener medición con todas las mejoras aplicadas
        err = getMappingValue(&angle, &distance);
        if (err == ESP_OK) {
            printf("Angle: %d°, Distance: %d mm\n", angle, distance);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## Funcionalidades Avanzadas (Opcionales)

### 1. Compensación de Movimiento
Si tu robot tiene sensores de velocidad, puedes habilitar la compensación de movimiento:

```c
// En tu código principal
mapping_set_robot_motion(velocity_x, velocity_y, angular_velocity);
mapping_set_motion_compensation(true);
```

### 2. Calibración Manual
Para calibración con distancia conocida:

```c
// Calibrar con un objeto a 1000mm de distancia
mapping_calibrate(1000);
```

### 3. Estadísticas del Sistema
Para monitorear el rendimiento:

```c
mapping_statistics_t stats;
mapping_get_statistics(&stats);
printf("Success rate: %d%%, Avg quality: %d%%\n", 
       stats.success_rate_percent, stats.avg_quality_score);
```

### 4. Mediciones Interpoladas
Para obtener mediciones en ángulos específicos:

```c
mapping_measurement_t measurement;
esp_err_t err = mapping_get_interpolated_measurement(45, &measurement);
if (err == ESP_OK) {
    printf("Interpolated at 45°: %d mm\n", measurement.distance_mm);
}
```

## Logs y Diagnóstico

### Logs de Inicialización
```
I (1234) MAPPING: Initializing mapping system with automatic enhancement detection
I (1235) MAPPING: Performing automatic environment analysis...
I (1345) MAPPING: Environment analysis: Avg quality: 75%, Avg signal: 1200
I (1346) MAPPING: Excellent environment detected, optimizing for speed
I (1347) MAPPING: Performing automatic calibration...
I (1456) MAPPING: Auto-calibration completed. Median distance: 1500 mm, Estimated offset: 10 mm
I (1457) MAPPING: Mapping system initialized with enhanced features enabled
I (1458) MAPPING: Features: Continuous mode, filtering, quality assessment, interpolation, auto-calibration
```

### Logs de Calidad (si está habilitado)
```
D (5000) MAPPING: Measurement quality: 85%, Signal: 1250, Raw: 1495mm, Filtered: 1505mm
D (10000) MAPPING: Measurement quality: 82%, Signal: 1180, Raw: 1502mm, Filtered: 1512mm
```

## Optimización de Rendimiento

### Para Entornos de Alta Velocidad
```c
// En mapping_config.h
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_20MS
#define MAPPING_ANALYSIS_SAMPLES 15
#define MAPPING_CALIBRATION_SAMPLES 8
```

### Para Máxima Precisión
```c
// En mapping_config.h
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_100MS
#define MAPPING_ANALYSIS_SAMPLES 30
#define MAPPING_CALIBRATION_SAMPLES 15
#define MAPPING_CALIBRATION_MIN_QUALITY 70
```

## Solución de Problemas

### Problema: Mediciones inconsistentes
**Solución**: Habilitar más filtrado
```c
#define MAPPING_ANALYSIS_SAMPLES 25
#define MAPPING_CALIBRATION_MIN_QUALITY 60
```

### Problema: Inicialización lenta
**Solución**: Reducir muestras de análisis
```c
#define MAPPING_ANALYSIS_SAMPLES 10
#define MAPPING_CALIBRATION_SAMPLES 5
```

### Problema: Calidad de medición baja
**Solución**: Aumentar presupuesto de tiempo
```c
#define MAPPING_ANALYSIS_TIMING_BUDGET VL53L0X_TIMING_BUDGET_100MS
```

## Compatibilidad

- **Totalmente compatible** con código existente
- **No requiere cambios** en `main.c`
- **Funciones legacy** (`mapping_init`, `getMappingValue`) funcionan igual
- **Mejoras automáticas** se aplican internamente

## Ventajas

1. **Cero cambios en main.c** - Todo funciona automáticamente
2. **Configuración flexible** - Personalizable sin modificar código
3. **Mejor rendimiento** - Hasta 50Hz vs 10Hz original
4. **Mayor precisión** - Filtrado y calibración automática
5. **Robustez** - Interpolación y fallbacks automáticos
6. **Diagnóstico** - Logging detallado para debugging

## Próximas Mejoras

- [ ] Detección automática de velocidad del servo
- [ ] Ajuste adaptativo de parámetros en tiempo real
- [ ] Almacenamiento de configuraciones en NVS
- [ ] Interfaz web para configuración remota
- [ ] Integración con sensores IMU para compensación de movimiento 