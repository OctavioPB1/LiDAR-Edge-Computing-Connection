/**
 * Utilidades para transformación de coordenadas
 */

export interface PolarCoordinate {
  distance: number;
  angle: number; // en grados
}

export interface CartesianCoordinate {
  x: number;
  y: number;
}

/**
 * Convierte coordenadas polares (distancia, ángulo) a coordenadas cartesianas (X, Y)
 * 
 * @param distance - Distancia desde el origen
 * @param angle - Ángulo en grados (0° = este, 90° = norte, 180° = oeste, 270° = sur)
 * @returns Coordenadas cartesianas {x, y}
 */
export function polarToCartesian(distance: number, angle: number): CartesianCoordinate {
  // Convertir grados a radianes
  const radians = (angle * Math.PI) / 180;
  
  // Calcular coordenadas cartesianas
  const x = distance * Math.cos(radians);
  const y = distance * Math.sin(radians);
  
  return { x, y };
}

/**
 * Convierte coordenadas polares a cartesianas usando interfaz de objeto
 * 
 * @param polar - Objeto con coordenadas polares
 * @returns Coordenadas cartesianas {x, y}
 */
export function polarObjectToCartesian(polar: PolarCoordinate): CartesianCoordinate {
  return polarToCartesian(polar.distance, polar.angle);
}

/**
 * Convierte múltiples coordenadas polares a cartesianas
 * 
 * @param polarPoints - Array de coordenadas polares
 * @returns Array de coordenadas cartesianas
 */
export function polarArrayToCartesian(polarPoints: PolarCoordinate[]): CartesianCoordinate[] {
  return polarPoints.map(point => polarToCartesian(point.distance, point.angle));
}

/**
 * Convierte coordenadas polares a cartesianas con escala y offset para visualización
 * 
 * @param distance - Distancia desde el origen
 * @param angle - Ángulo en grados
 * @param scale - Factor de escala para convertir unidades a píxeles
 * @param centerX - Coordenada X del centro (offset)
 * @param centerY - Coordenada Y del centro (offset)
 * @param invertY - Si invertir el eje Y (típico para SVG/Canvas)
 * @returns Coordenadas cartesianas escaladas y centradas
 */
export function polarToCartesianScaled(
  distance: number, 
  angle: number, 
  scale: number = 1, 
  centerX: number = 0, 
  centerY: number = 0, 
  invertY: boolean = false
): CartesianCoordinate {
  const { x, y } = polarToCartesian(distance, angle);
  
  return {
    x: centerX + (x * scale),
    y: centerY + (invertY ? -(y * scale) : (y * scale))
  };
} 