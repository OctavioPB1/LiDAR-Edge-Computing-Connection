import { Component, OnInit, OnDestroy, ElementRef } from '@angular/core';
import { MappingValueService } from '../../../core/services/mapping-value.service';
import * as d3 from 'd3';
import { polarToCartesianScaled, CartesianCoordinate } from '../../utils/coordinate-transform.util';


export interface Point {
  angle: number;
  distance: number;
}

/**
 * @component MapComponent
 * @description This component is responsible for managing the map view and visualizing the data received from the backend. 
 * It includes functionalities to display and interact with the map and process incoming data based on user inputs.
 * The component also handles the setup and control of the mapping process.
 *
 */
@Component({
  selector: 'app-map',
  standalone: true,
  imports: [],
  templateUrl: './map.component.html',
  styleUrl: './map.component.scss',
})

export class MapComponent implements OnInit, OnDestroy {
  private mapping: boolean = true;
  private width = 850;
  private height = 850;
  private svg: any;
  private canvas: HTMLCanvasElement | null = null;
  private ctx: CanvasRenderingContext2D | null = null;
  private maxDistance = 2.5; // Distancia máxima en metros (aumentada para LiDAR)
  private scaleFactor = this.width / (this.maxDistance * 0.9); // Escala para convertir metros a pixeles
  private pointsMap: Map<number, any> = new Map();
  private pointLifetime = 20 * 1000; // Tiempo en milisegundos antes de borrar un punto
  
  // Centro desplazado hacia abajo para simular sonar de barco
  private centerX = this.width / 2;
  private centerY = this.height * 0.7; // Desplazamiento hacia abajo

  private pointsToPlot: { distance: number; angle: number }[] = []; // Lista de puntos pendientes
  private readonly maxPointsPerFrame = 800; // Aumentado para Canvas
  private canvasPoints: Array<{x: number, y: number, timestamp: number}> = []; // Points for canvas
  private lastPlottedPoint: { x: number; y: number } | null = null; // Último punto dibujado para conectar
  private backendPollingInterval?: number;
  private animationFrameId?: number;
  
  // Optimizaciones de filtrado
  private readonly minPointDistance = 3; // Distancia mínima en píxeles entre puntos
  private lastFilteredPoint: { x: number; y: number } | null = null;
  private needsRender = false; // Flag para indicar si necesita renderizar

  constructor(private mappingValueService: MappingValueService) {}

  /**
   * Initializes the component by creating the chart, fetching points from the backend,
   * and periodically updating the visualization. Points are retrieved and stored before
   * being plotted at regular intervals.
   */
  ngOnInit() {
    this.createChart();
    this.backendPollingInterval = setInterval(() => this.receivePointsFromBackend(), 150) as any; // Más frecuente para Canvas
    this.startRenderLoop(); // Usar requestAnimationFrame para rendering
  }

  /**
   * Cleanup method to prevent memory leaks
   */
  ngOnDestroy(): void {
    this.mapping = false;
    
    if (this.backendPollingInterval) {
      clearInterval(this.backendPollingInterval);
    }
    
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
    
    // Limpiar Canvas
    if (this.ctx && this.canvas) {
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }
    this.canvasPoints = [];
    
    // Limpiar todos los puntos del mapa SVG (si los hay)
    this.pointsMap.forEach((point) => {
      point.remove();
    });
    this.pointsMap.clear();
    this.lastPlottedPoint = null;
    this.lastFilteredPoint = null;
    this.pointsToPlot = [];
    this.needsRender = false;
  }

  /**
   * Starts the render loop using requestAnimationFrame for smoother performance
   * Only renders when there are actual changes to improve performance
   */
  private startRenderLoop(): void {
    const render = () => {
      if (this.mapping) {
        if (this.pointsToPlot.length > 0) {
          this.plotStoredPoints();
          this.needsRender = true;
        }
        
        // Solo renderizar si hay cambios o necesita limpieza
        if (this.needsRender) {
          this.cleanupOldPoints();
          this.needsRender = false;
        }
        
        this.animationFrameId = requestAnimationFrame(render);
      }
    };
    this.animationFrameId = requestAnimationFrame(render);
  }

  /**
   * Cleanup old points more efficiently using batch operations
   */
  private cleanupOldPoints(): void {
    const currentTime = Date.now();
    const initialLength = this.canvasPoints.length;
    
    this.canvasPoints = this.canvasPoints.filter(point => 
      currentTime - point.timestamp < this.pointLifetime
    );
    
    // Solo re-renderizar si se eliminaron puntos
    if (this.canvasPoints.length !== initialLength) {
      this.renderCanvasPoints();
    }
  }

  /**
   * Fetches mapping values from the backend and stores them in a list for later visualization.
   * This function is periodically triggered to ensure new points are continuously added.
   * Uses dynamic throttling based on current buffer size for optimal performance.
   */
  receivePointsFromBackend(): void {
    // Throttling dinámico: reducir requests si hay muchos puntos pendientes
    const bufferRatio = this.pointsToPlot.length / 500;
    if (!this.mapping || bufferRatio > 0.9) {
      console.log("Salteando...");
      return; // Saltar request si buffer está muy lleno
    }

    this.mappingValueService.getMappingValues().subscribe({
      next: (data) => {
        if (Array.isArray(data) && data.length > 0) {
          // Filtrar datos en el backend para reducir transferencia
          const filteredData = data.filter((point, index) => 
            index % Math.max(1, Math.floor(bufferRatio * 3)) === 0 // Saltear puntos si hay mucha carga
          );
          
          // Agregar puntos en lote para mejor rendimiento
          this.pointsToPlot.push(...filteredData);
          
          // Limitar el buffer para evitar acumulación excesiva
          if (this.pointsToPlot.length > 600) {
            this.pointsToPlot = this.pointsToPlot.slice(-500); // Mantener solo los últimos 500
            console.log("Buffer lleno, eliminando puntos...");
          }
        }
      },
      error: (error) => {
        console.warn('Error fetching mapping data:', error);
      }
    });
  }
    

  /**
   * Plots stored points on the map using Canvas for better performance.
   * After plotting, the points are removed from the list to prevent duplication.
   * Each point remains visible for a defined lifetime before being removed.
   */ 
  plotStoredPoints(): void {
    if (!this.pointsToPlot.length || !this.mapping || !this.ctx) {
      return; // Nada que graficar
    }

    // Procesar un batch de puntos por frame
    const numPointsToProcess = Math.min(this.maxPointsPerFrame, this.pointsToPlot.length);
    const pointsToProcess = this.pointsToPlot.splice(0, numPointsToProcess);
    const currentTime = Date.now();

    // Convertir puntos a coordenadas cartesianas y agregar al array de Canvas
    pointsToProcess.forEach((point) => {
      let meters = point.distance / 1000; // Convertir mm a metros
      
      // Filtrar puntos fuera del rango máximo
      if (meters > this.maxDistance) {
        meters = this.maxDistance;
        return;
      }

      const { x, y } = this.robotCoordinatesToChart(meters, point.angle);
      
      // Filtro de distancia: solo agregar si está lo suficientemente lejos del último punto
      if (this.lastFilteredPoint) {
        const distance = Math.sqrt(
          Math.pow(x - this.lastFilteredPoint.x, 2) + 
          Math.pow(y - this.lastFilteredPoint.y, 2)
        );
        
        if (distance < this.minPointDistance) {
          return; // Omitir punto muy cercano
        }
      }
      
      // Agregar punto al array de Canvas con timestamp
      this.canvasPoints.push({
        x: x,
        y: y,
        timestamp: currentTime
      });

      // Actualizar último punto para líneas y filtrado
      this.lastPlottedPoint = { x, y };
      this.lastFilteredPoint = { x, y };
    });

    // Renderizar todos los puntos en Canvas
    this.renderCanvasPoints();
  }

  /**
   * Renders all points and lines on Canvas for optimal performance
   */
  private renderCanvasPoints(): void {
    if (!this.ctx || !this.canvas) return;

    // Limpiar canvas
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    
    // Configurar estilo
    this.ctx.fillStyle = 'black';
    this.ctx.strokeStyle = 'black';
    this.ctx.lineWidth = 4;

    // Dibujar líneas conectando puntos consecutivos
    if (this.canvasPoints.length > 1) {
      this.ctx.beginPath();
      this.ctx.moveTo(this.canvasPoints[0].x, this.canvasPoints[0].y);
      
      for (let i = 1; i < this.canvasPoints.length; i++) {
        this.ctx.lineTo(this.canvasPoints[i].x, this.canvasPoints[i].y);
      }
      
      this.ctx.stroke();
    }

    // Dibujar puntos
    this.canvasPoints.forEach(point => {
      this.ctx!.beginPath();
      this.ctx!.arc(point.x, point.y, 2, 0, 2 * Math.PI);
      this.ctx!.fill();
    });
  }



  /**
 * Sets the mapping process state.
  * This function updates the `mapping` variable, controlling whether the mapping process is active or paused.
  * 
  * @param mapping_process - A boolean indicating whether to start or stop the mapping process.
  */
  setup_mapping(mapping_process: boolean){
    this.mapping = mapping_process;
  }

  /**
   * Creates and initializes the chart with SVG elements for static content and Canvas for dynamic points.
   * It sets the dimensions of the chart, adds a white background, 
   * draws a border around the content area, and places the robot in the center of the chart.
   */
  createChart() {
    // Crear contenedor div para SVG y Canvas
    const chartContainer = d3.select('#chart');
    
    // Crear SVG para elementos estáticos (primero, como base)
    this.svg = chartContainer
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('position', 'absolute')
      .style('top', '0')
      .style('left', '0')
      .style('background', '#ffffff'); // Fondo blanco

    // Crear Canvas para puntos dinámicos (encima del SVG)
    this.canvas = chartContainer
      .append('canvas')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('position', 'absolute')
      .style('top', '0')
      .style('left', '0')
      .style('pointer-events', 'none') // Permitir que los eventos pasen al SVG debajo
      .node() as HTMLCanvasElement;
    
    this.ctx = this.canvas.getContext('2d');

    // Crear borde alrededor del contenido
    this.svg
      .append('rect')
      .attr('x', 0)
      .attr('y', 0)
      .attr('width', this.width)
      .attr('height', this.height)
      .attr('fill', 'none')
      .attr('stroke', '#213A7D')
      .attr('stroke-width', 16);

    // Definir los puntos del triángulo usando las nuevas coordenadas del centro
    const triangleSize = 20;

    const points = [
        [this.centerX, this.centerY - triangleSize], // Punta arriba
        [this.centerX - triangleSize, this.centerY + triangleSize], // Esquina inferior izquierda
        [this.centerX + triangleSize, this.centerY + triangleSize]  // Esquina inferior derecha
    ].map(p => p.join(',')).join(' ');

    // Dibujar el triángulo en el centro
    this.svg
      .append('polygon')
      .attr('points', points)
      .attr('fill', '#213A7D');

    // Dibujar las circunferencias con los radios dados y sus etiquetas
    const radii = [0.5, 1.0, 1.5, 2.0]; // Radios en metros (ajustados para maxDistance = 2m)
    radii.forEach(radius => {
        // Dibujar círculo
        this.svg
          .append('circle')
          .attr('cx', this.centerX)
          .attr('cy', this.centerY)
          .attr('r', radius * this.scaleFactor) // Escalar el radio
          .attr('fill', 'none')
          .attr('stroke', '#213A7D')
          .attr('stroke-width', 2);

        // Agregar etiqueta de distancia
        this.svg
          .append('text')
          .attr('x', this.centerX + radius * this.scaleFactor + 5) // Posicionar a la derecha del círculo
          .attr('y', this.centerY + 5) // Centrar verticalmente con un pequeño offset
          .attr('fill', '#213A7D')
          .attr('font-size', '14px')
          .attr('font-weight', 'bold')
          .text(`${radius}m`);
    });
}


  /**
   * Clears all stored points on the map by removing their corresponding graphical elements 
   * from the SVG and Canvas, and resetting the points maps.
   */
  restartMapping(){
    // Limpiar Canvas
    if (this.ctx && this.canvas) {
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }
    this.canvasPoints = [];
    
    // Recorrer todos los puntos almacenados en el mapa SVG (si los hay)
    this.pointsMap.forEach((point) => {
      point.remove(); // Eliminar el elemento gráfico del SVG
    });

    // Limpiar el mapa de puntos y resetear último punto
    this.pointsMap.clear();
    this.lastPlottedPoint = null;
    this.lastFilteredPoint = null;
    this.pointsToPlot = [];
    this.needsRender = false;
  }
  

  /**
   * Convierte coordenadas polares del robot a coordenadas para el gráfico
   * Usa la función utilitaria polarToCartesianScaled para mayor reutilización
   * 
   * @param distance La distancia desde el origen (en metros)
   * @param angle El ángulo en grados
   * @returns Las coordenadas cartesianas (x, y) para el SVG
   */
  robotCoordinatesToChart(distance: number, angle: number): CartesianCoordinate {
    return polarToCartesianScaled(
      distance, 
      angle, // Ajuste para que 0° sea hacia adelante del robot
      this.scaleFactor, 
      this.centerX, 
      this.centerY, 
      true // Invertir Y para SVG
    );
  }


  /**
   * Captures the current SVG map and Canvas points as an image, including a border around it. 
   * The image is saved as a PNG file with the border added around the map for clarity.
   */
  captureMap() {
    const svgElement = this.svg.node() as SVGSVGElement;
    const serializer = new XMLSerializer();
    const svgString = serializer.serializeToString(svgElement);
  
    // Dimensiones con borde incluido
    const borderSize = 8; // Tamaño del borde
    const canvas = document.createElement('canvas');
    canvas.width = this.width + 2 * borderSize;
    canvas.height = this.height + 2 * borderSize;
    const context = canvas.getContext('2d');
  
    // Dibujar el borde uniformemente
    context!.fillStyle = '#213A7D'; // Color del borde
    context!.fillRect(0, 0, canvas.width, canvas.height);
  
    const svgImage = new Image();
    svgImage.onload = () => {
      // Dibujar la imagen SVG centrada dentro del borde
      context?.drawImage(svgImage, borderSize, borderSize, this.width, this.height);
      
      // Dibujar el contenido del Canvas encima del SVG
      if (this.canvas) {
        context?.drawImage(this.canvas, borderSize, borderSize, this.width, this.height);
      }
      
      const dataURL = canvas.toDataURL('image/png');
  
      // Descargar la imagen
      const link = document.createElement('a');
      link.href = dataURL;
      link.download = 'map-capture-with-borders.png';
      link.click();
    };
    svgImage.src = `data:image/svg+xml;base64,${btoa(svgString)}`;
  }
  
  

}
