# Librería Genérica de Control de Servomotores para ESP32

Esta librería proporciona una interfaz unificada para controlar diferentes tipos de servomotores en el ESP32, incluyendo servomotores de posición y de giro continuo.

## Características

- ✅ **Soporte para múltiples tipos de servo**: Posición (0-180°), giro continuo, y configuración personalizada
- ✅ **Configuraciones predefinidas**: Para servomotores comunes (SG90, MG996R, Futaba, etc.)
- ✅ **Control de velocidad**: Para servomotores de giro continuo (0-100%)
- ✅ **Control de posición**: Para servomotores estándar (0-180° o personalizado)
- ✅ **Movimiento suave**: Transiciones graduales entre posiciones
- ✅ **Operaciones thread-safe**: Seguro para uso en múltiples tareas
- ✅ **Manejo de errores**: Validación de parámetros y estados
- ✅ **Inversión de dirección**: Configurable para adaptarse a diferentes montajes
- ✅ **Control de pulso personalizado**: Para casos especiales

## Tipos de Servomotores Soportados

### 1. Servomotores de Posición
- **Rango estándar**: 0-180 grados
- **Rango extendido**: 0-270 grados (configurable)
- **Ejemplos**: SG90, MG996R, TowerPro

### 2. Servomotores de Giro Continuo
- **Control de velocidad**: 0-100%
- **Dirección**: Horario y antihorario
- **Ejemplos**: Futaba, Hitec, TowerPro

### 3. Servomotores Personalizados
- **Configuración flexible**: Parámetros ajustables
- **Rangos personalizados**: Ángulos y pulsos definibles

## Configuraciones Predefinidas

```c
// Servomotores de posición
extern const servo_config_t SERVO_CONFIG_POSITION_STANDARD;  // Estándar 0-180°
extern const servo_config_t SERVO_CONFIG_POSITION_MG996R;    // MG996R
extern const servo_config_t SERVO_CONFIG_POSITION_SG90;      // SG90

// Servomotores de giro continuo
extern const servo_config_t SERVO_CONFIG_CONTINUOUS_STANDARD; // Estándar
extern const servo_config_t SERVO_CONFIG_CONTINUOUS_FUTABA;   // Futaba
```

## Uso Básico

### 1. Inicialización

```c
#include "servo_generic.h"

// Para servomotor de posición
servo_handle_t *servo = NULL;
esp_err_t ret = servo_generic_init(&SERVO_CONFIG_POSITION_STANDARD, &servo);
if (ret != ESP_OK) {
    // Manejar error
}

// Habilitar el servo
servo_generic_enable(servo);
```

### 2. Control de Posición

```c
// Mover a 0 grados
servo_generic_set_position(servo, 0);

// Mover a 90 grados
servo_generic_set_position(servo, 90);

// Mover a 180 grados
servo_generic_set_position(servo, 180);

// Movimiento suave (3 segundos)
servo_generic_move_smooth(servo, 0, 3000);
```

### 3. Control de Velocidad (Giro Continuo)

```c
// Rotar en sentido horario al 50%
servo_generic_set_speed(servo, 50, SERVO_DIR_CW);

// Rotar en sentido antihorario al 75%
servo_generic_set_speed(servo, 75, SERVO_DIR_CCW);

// Detener
servo_generic_stop(servo);
```

### 4. Configuración Personalizada

```c
// Crear configuración personalizada
servo_config_t custom_config = servo_generic_create_config(
    GPIO_NUM_14,           // Pin GPIO
    SERVO_TYPE_POSITION,   // Tipo de servo
    600,                   // Pulso mínimo (600µs)
    2400,                  // Pulso máximo (2400µs)
    1500,                  // Pulso central (1500µs)
    50,                    // Frecuencia (50Hz)
    0,                     // Ángulo mínimo
    270                    // Ángulo máximo (270°)
);

servo_handle_t *custom_servo = NULL;
servo_generic_init(&custom_config, &custom_servo);
```

## API Completa

### Funciones de Inicialización

```c
esp_err_t servo_generic_init(const servo_config_t *config, servo_handle_t **handle);
esp_err_t servo_generic_deinit(servo_handle_t *handle);
esp_err_t servo_generic_enable(servo_handle_t *handle);
esp_err_t servo_generic_disable(servo_handle_t *handle);
```

### Funciones de Control

```c
// Para servomotores de posición
esp_err_t servo_generic_set_position(servo_handle_t *handle, uint16_t angle);
esp_err_t servo_generic_move_smooth(servo_handle_t *handle, uint16_t target_angle, uint32_t duration_ms);

// Para servomotores de giro continuo
esp_err_t servo_generic_set_speed(servo_handle_t *handle, uint8_t speed, servo_direction_t direction);
esp_err_t servo_generic_set_speed_direction(servo_handle_t *handle, uint8_t speed, servo_direction_t direction);

// Funciones generales
esp_err_t servo_generic_stop(servo_handle_t *handle);
esp_err_t servo_generic_set_pulse(servo_handle_t *handle, uint32_t pulse_width);
esp_err_t servo_generic_invert_direction(servo_handle_t *handle);
```

### Funciones de Consulta

```c
uint16_t servo_generic_get_position(servo_handle_t *handle);
uint8_t servo_generic_get_speed(servo_handle_t *handle);
uint32_t servo_generic_get_pulse(servo_handle_t *handle);
bool servo_generic_is_enabled(servo_handle_t *handle);
```

### Funciones de Configuración

```c
servo_config_t servo_generic_create_config(gpio_num_t pin, servo_type_t type, 
                                          uint32_t min_pulse, uint32_t max_pulse, uint32_t center_pulse,
                                          uint32_t frequency, uint16_t min_angle, uint16_t max_angle);
```

## Estructuras de Datos

### servo_config_t
```c
typedef struct {
    gpio_num_t pin;                  // Pin GPIO para señal PWM
    servo_type_t type;               // Tipo de servomotor
    uint32_t min_pulse_us;           // Ancho de pulso mínimo (microsegundos)
    uint32_t max_pulse_us;           // Ancho de pulso máximo (microsegundos)
    uint32_t center_pulse_us;        // Ancho de pulso central/parada (microsegundos)
    uint32_t frequency_hz;           // Frecuencia PWM (Hz, típicamente 50)
    uint16_t min_angle;              // Ángulo mínimo para servos de posición
    uint16_t max_angle;              // Ángulo máximo para servos de posición
    uint8_t speed;                   // Velocidad para giro continuo (0-100)
    bool invert_direction;           // Invertir dirección de rotación
} servo_config_t;
```

### Enumeraciones

```c
typedef enum {
    SERVO_TYPE_POSITION,     // Servomotor de posición estándar (0-180°)
    SERVO_TYPE_CONTINUOUS,   // Servomotor de giro continuo
    SERVO_TYPE_CUSTOM        // Servomotor con parámetros personalizados
} servo_type_t;

typedef enum {
    SERVO_DIR_CW,    // Rotación en sentido horario
    SERVO_DIR_CCW,   // Rotación en sentido antihorario
    SERVO_DIR_STOP   // Detener rotación
} servo_direction_t;
```

## Ejemplos de Uso

### Ejemplo 1: Servomotor de Posición Básico

```c
void servo_position_example(void)
{
    servo_handle_t *servo = NULL;
    
    // Inicializar con configuración estándar
    esp_err_t ret = servo_generic_init(&SERVO_CONFIG_POSITION_STANDARD, &servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando servo");
        return;
    }
    
    // Habilitar
    servo_generic_enable(servo);
    
    // Mover a diferentes posiciones
    servo_generic_set_position(servo, 0);    // 0°
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    servo_generic_set_position(servo, 90);   // 90°
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    servo_generic_set_position(servo, 180);  // 180°
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Movimiento suave de vuelta a 0°
    servo_generic_move_smooth(servo, 0, 3000);
    
    // Limpiar
    servo_generic_deinit(servo);
}
```

### Ejemplo 2: Servomotor de Giro Continuo

```c
void servo_continuous_example(void)
{
    servo_handle_t *servo = NULL;
    
    // Inicializar con configuración de giro continuo
    esp_err_t ret = servo_generic_init(&SERVO_CONFIG_CONTINUOUS_STANDARD, &servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando servo");
        return;
    }
    
    // Habilitar
    servo_generic_enable(servo);
    
    // Rotar en sentido horario al 50%
    servo_generic_set_speed(servo, 50, SERVO_DIR_CW);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Detener
    servo_generic_stop(servo);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Rotar en sentido antihorario al 75%
    servo_generic_set_speed(servo, 75, SERVO_DIR_CCW);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Detener
    servo_generic_stop(servo);
    
    // Limpiar
    servo_generic_deinit(servo);
}
```

### Ejemplo 3: Múltiples Servomotores

```c
void multiple_servos_example(void)
{
    servo_handle_t *pos_servo = NULL;
    servo_handle_t *cont_servo = NULL;
    
    // Configurar servomotor de posición en GPIO 14
    servo_config_t pos_config = SERVO_CONFIG_POSITION_STANDARD;
    pos_config.pin = GPIO_NUM_14;
    servo_generic_init(&pos_config, &pos_servo);
    
    // Configurar servomotor de giro continuo en GPIO 15
    servo_config_t cont_config = SERVO_CONFIG_CONTINUOUS_STANDARD;
    cont_config.pin = GPIO_NUM_15;
    servo_generic_init(&cont_config, &cont_servo);
    
    // Habilitar ambos
    servo_generic_enable(pos_servo);
    servo_generic_enable(cont_servo);
    
    // Controlar ambos simultáneamente
    servo_generic_set_position(pos_servo, 90);
    servo_generic_set_speed(cont_servo, 50, SERVO_DIR_CW);
    
    // Limpiar
    servo_generic_deinit(pos_servo);
    servo_generic_deinit(cont_servo);
}
```

## Consideraciones Técnicas

### Frecuencia PWM
- **Estándar**: 50Hz (20ms período)
- **Configurable**: 20-100Hz según el servo
- **Resolución**: 1µs por tick

### Anchos de Pulso Típicos
- **Servomotores de posición**: 500-2500µs
- **Servomotores de giro continuo**: 900-2100µs
- **Pulso central/parada**: 1500µs

### Consumo de Energía
- **Corriente de reposo**: 5-10mA
- **Corriente bajo carga**: 100-500mA
- **Pico de corriente**: Hasta 1A durante arranque

## Solución de Problemas

### Error: "Failed to create MCPWM timer"
- Verificar que el pin GPIO no esté en uso
- Comprobar que el grupo MCPWM esté disponible

### Error: "Angle out of range"
- Verificar la configuración de ángulos mín/máx
- Asegurar que el ángulo solicitado esté dentro del rango

### Error: "Speed out of range"
- La velocidad debe estar entre 0-100%
- Verificar que el servo sea de tipo continuo

### El servo no se mueve
- Verificar conexiones de alimentación (5V)
- Comprobar conexión de señal PWM
- Verificar que el servo esté habilitado

### Movimiento errático
- Verificar frecuencia PWM (debe ser 50Hz)
- Comprobar ancho de pulso configurado
- Ajustar configuración según el modelo específico

## Compatibilidad

### Servomotores Probados
- ✅ SG90 (TowerPro)
- ✅ MG996R (TowerPro)
- ✅ Futaba S3003
- ✅ Hitec HS-311
- ✅ Servomotores genéricos de 9g

### Plataformas Soportadas
- ✅ ESP32 (todas las variantes)
- ✅ ESP32-S2
- ✅ ESP32-S3
- ✅ ESP32-C3

## Licencia

Esta librería está bajo licencia MIT. Ver archivo LICENSE para más detalles.

## Contribuciones

Las contribuciones son bienvenidas. Por favor:
1. Fork el repositorio
2. Crea una rama para tu feature
3. Commit tus cambios
4. Push a la rama
5. Crea un Pull Request

## Changelog

### v2.0 (2025-01-27)
- ✅ Librería completamente reescrita
- ✅ Soporte para múltiples tipos de servo
- ✅ Configuraciones predefinidas
- ✅ API unificada y thread-safe
- ✅ Documentación completa

### v1.0 (2024-12-05)
- ✅ Implementación básica para servomotor de giro continuo
- ✅ Control de velocidad y dirección
- ✅ Integración con limit switch 