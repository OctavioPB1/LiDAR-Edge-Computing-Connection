# Guía de Migración: Librería de Servomotores

## 📋 Resumen

Esta guía te ayudará a migrar desde la librería de servomotores actual (`servo.h`/`servo.c`) a la nueva librería genérica (`servo_generic.h`/`servo_generic.c`) de manera segura y sin romper el código existente.

## 🎯 ¿Por qué migrar?

### **Ventajas de la nueva librería:**

✅ **Más flexible**: Soporta servomotores de posición y giro continuo  
✅ **Mejor API**: Funciones más claras y consistentes  
✅ **Configuraciones predefinidas**: Para servomotores comunes (SG90, MG996R, etc.)  
✅ **Thread-safe**: Mejor manejo de concurrencia  
✅ **Mejor manejo de errores**: Validaciones más robustas  
✅ **Documentación completa**: Con ejemplos y README  

### **Compatibilidad total**: Tu código actual seguirá funcionando sin cambios

## 🚀 Opciones de Migración

### **Opción 1: Migración Gradual (Recomendada)**

Usa la capa de compatibilidad que mantiene tu API actual:

```c
// Cambia solo el include
#include "servo_compatibility.h"  // En lugar de "servo.h"

// Tu código actual sigue funcionando igual
servo_initialize();
servo_start();
servo_set_speed(UP);
servo_stop();
```

### **Opción 2: Migración Directa**

Usa directamente la nueva API genérica:

```c
#include "Generic_servo/servo_generic.h"

// Inicialización
servo_handle_t *servo = NULL;
servo_generic_init(&SERVO_CONFIG_CONTINUOUS_STANDARD, &servo);
servo_generic_enable(servo);

// Control
servo_generic_set_speed(servo, 50, SERVO_DIR_CCW);  // 50% velocidad CCW
servo_generic_stop(servo);
```

## 📁 Archivos de la Migración

### **Archivos Nuevos:**
- `Generic_servo/servo_generic.h` - Nueva librería genérica
- `Generic_servo/servo_generic.c` - Implementación
- `Generic_servo/servo_generic_example.c` - Ejemplos de uso
- `Generic_servo/README_SERVO_GENERIC.md` - Documentación completa
- `servo_compatibility.h` - Capa de compatibilidad
- `servo_compatibility.c` - Implementación de compatibilidad

### **Archivos Actuales (se pueden eliminar después de la migración):**
- `servo.h` - API antigua
- `servo.c` - Implementación antigua

## 🔄 Pasos para la Migración

### **Paso 1: Migración Gradual (Recomendada)**

1. **Reemplaza el include en tus archivos:**
   ```c
   // Cambiar de:
   #include "servo.h"
   
   // A:
   #include "servo_compatibility.h"
   ```

2. **Compila y prueba:**
   ```bash
   pio run
   ```

3. **Verifica que todo funciona igual:**
   - El servo debe inicializarse correctamente
   - Los comandos de velocidad deben funcionar
   - Las funciones de pausa/continuar deben funcionar

### **Paso 2: Migración a la Nueva API (Opcional)**

Una vez que la migración gradual funcione, puedes migrar gradualmente a la nueva API:

```c
// Código actual:
servo_initialize();
servo_start();
servo_set_speed(UP);

// Nuevo código:
servo_handle_t *servo = NULL;
servo_generic_init(&SERVO_CONFIG_CONTINUOUS_STANDARD, &servo);
servo_generic_enable(servo);
servo_generic_set_speed(servo, 75, SERVO_DIR_CCW);  // 75% velocidad CCW
```

## 🔧 Configuraciones Disponibles

### **Servomotores de Posición:**
```c
SERVO_CONFIG_POSITION_STANDARD  // 0-180°, 500-2500µs
SERVO_CONFIG_POSITION_MG996R    // 0-180°, 600-2400µs  
SERVO_CONFIG_POSITION_SG90      // 0-180°, 500-2500µs
```

### **Servomotores de Giro Continuo:**
```c
SERVO_CONFIG_CONTINUOUS_STANDARD  // 900-2100µs (como tu servo actual)
SERVO_CONFIG_CONTINUOUS_FUTABA    // 1000-2000µs
```

### **Configuración Personalizada:**
```c
servo_config_t custom_config = {
    .pin = GPIO_NUM_14,
    .type = SERVO_TYPE_CONTINUOUS,
    .min_pulse_us = 900,
    .max_pulse_us = 2100,
    .center_pulse_us = 1500,
    .frequency_hz = 50,
    .speed = 50,
    .invert_direction = false
};
```

## 📊 Mapeo de Funciones

### **API Actual → Nueva API:**

| Función Actual | Función Nueva | Notas |
|---|---|---|
| `servo_initialize()` | `servo_generic_init()` | Configuración automática |
| `servo_start()` | `servo_generic_enable()` + `servo_generic_set_speed()` | Inicia con velocidad media |
| `servo_stop()` | `servo_generic_stop()` | Detiene el servo |
| `servo_pause()` | `servo_generic_stop()` | Pausa temporal |
| `servo_restart()` | `servo_generic_set_speed()` | Restaura velocidad anterior |
| `servo_set_speed(UP/DOWN)` | `servo_generic_set_speed()` | Control de velocidad |
| `servo_invert()` | Configuración `invert_direction` | Inversión de dirección |
| `readAngle()` | `servo_generic_get_position()` | Solo para servos de posición |

## 🧪 Pruebas de Migración

### **Test 1: Funcionalidad Básica**
```c
// Debe funcionar igual que antes
servo_initialize();
servo_start();
vTaskDelay(2000 / portTICK_PERIOD_MS);
servo_stop();
```

### **Test 2: Control de Velocidad**
```c
servo_initialize();
servo_start();
servo_set_speed(UP);   // Aumentar velocidad
vTaskDelay(1000 / portTICK_PERIOD_MS);
servo_set_speed(DOWN); // Disminuir velocidad
vTaskDelay(1000 / portTICK_PERIOD_MS);
servo_stop();
```

### **Test 3: Pausa y Continuar**
```c
servo_initialize();
servo_start();
vTaskDelay(1000 / portTICK_PERIOD_MS);
servo_pause();
vTaskDelay(2000 / portTICK_PERIOD_MS);
servo_restart();
vTaskDelay(1000 / portTICK_PERIOD_MS);
servo_stop();
```

## ⚠️ Consideraciones Importantes

### **1. Compatibilidad de Hardware**
- La nueva librería usa la misma configuración de hardware que la actual
- GPIO 14, 50Hz, pulsos de 900-2100µs
- No se requieren cambios en el hardware

### **2. Thread Safety**
- La nueva librería es thread-safe por diseño
- Los semáforos se mantienen para compatibilidad
- Mejor manejo de concurrencia

### **3. Manejo de Errores**
- La nueva librería tiene mejor validación de parámetros
- Códigos de error más específicos
- Logging mejorado para debugging

### **4. Rendimiento**
- La nueva librería es más eficiente
- Menos overhead en las operaciones
- Mejor gestión de memoria

## 🎉 Beneficios Post-Migración

### **Inmediatos:**
- ✅ Código más robusto y mantenible
- ✅ Mejor documentación y ejemplos
- ✅ Funcionalidad expandida

### **A Largo Plazo:**
- ✅ Fácil integración de nuevos tipos de servo
- ✅ API consistente para todo el proyecto
- ✅ Mejor testing y debugging
- ✅ Preparado para futuras expansiones

## 🆘 Solución de Problemas

### **Problema: El servo no responde**
```c
// Verificar inicialización
esp_err_t ret = servo_initialize();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error: %s", esp_err_to_name(ret));
}
```

### **Problema: Velocidad incorrecta**
```c
// Usar la nueva API para control directo
servo_handle_t *servo = servo_get_generic_handle();
servo_generic_set_speed(servo, 50, SERVO_DIR_CCW);
```

### **Problema: Dirección invertida**
```c
// Configurar inversión de dirección
servo_config_t config = SERVO_CONFIG_CONTINUOUS_STANDARD;
config.invert_direction = true;
```

## 📞 Soporte

Si encuentras problemas durante la migración:

1. **Revisa los logs**: La nueva librería tiene mejor logging
2. **Usa la capa de compatibilidad**: Mantiene tu API actual
3. **Consulta los ejemplos**: `servo_generic_example.c`
4. **Lee la documentación**: `README_SERVO_GENERIC.md`

---

**¡La migración es opcional pero recomendada!** Tu código actual seguirá funcionando mientras decides si migrar o no. 