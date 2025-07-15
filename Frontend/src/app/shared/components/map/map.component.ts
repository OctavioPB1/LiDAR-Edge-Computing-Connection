import { Component, OnInit, ElementRef } from '@angular/core';
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

export class MapComponent implements OnInit {
  private mapping: boolean = true;
  private width = 850;
  private height = 850;
  private svg: any;
  private maxDistance = 2; // Distancia máxima en metros (aumentada para LiDAR)
  private scaleFactor = this.width / (this.maxDistance * 2); // Escala para convertir metros a pixeles
  private pointsMap: Map<number, any> = new Map();
  private pointLifetime = 2000; // Tiempo en milisegundos antes de borrar un punto

  private pointsToPlot: { distance: number; angle: number }[] = []; // Lista de puntos pendientes
  //private pointsMap = new Map<string, any>(); // Mapa para graficar puntos únicos
  constructor(private mappingValueService: MappingValueService) {}

  /**
   * Initializes the component by creating the chart, fetching points from the backend,
   * and periodically updating the visualization. Points are retrieved and stored before
   * being plotted at regular intervals.
   */
    ngOnInit() {
      this.createChart();
     setInterval(() => this.receivePointsFromBackend(), 1000); // Llama cada 1 segundo
     setInterval(() => this.plotStoredPoints(), 50); // Graficar cada 50ms (más eficiente)
    }

  /**
   * Fetches mapping values from the backend and stores them in a list for later visualization.
   * This function is periodically triggered to ensure new points are continuously added.
   */
    receivePointsFromBackend(): void {
      this.mappingValueService.getMappingValues().subscribe((data) => {
    
        if (Array.isArray(data) && data.length > 0) {
          data.forEach((point) => {
            this.pointsToPlot.push(point); // Agrega cada punto individualmente
          });
        } else if (data && data.length != 0) {
          console.warn('Datos inesperados recibidos del backend:', data);
        }
      });
    }
    

  /**
   * Plots stored points on the map by converting their polar coordinates to Cartesian.
   * After plotting, the points are removed from the list to prevent duplication.
   * Each point remains visible for a defined lifetime before being removed.
   */ 
  plotStoredPoints(): void {
    
    if (!this.pointsToPlot.length || !this.mapping) {
      return; // Nada que graficar
    }

    this.pointsToPlot.forEach((point) => {
      // Convertir de polar (ángulo, distancia) a coordenadas cartesianas (x, y)
      // console.log("d,a: ",point.distance, point.angle)
      const meters = point.distance/250; // Convertir mm a metros correctamente
      
      // Filtrar puntos fuera del rango máximo
      if (meters > this.maxDistance) {
        // console.log(`Punto descartado: ${meters}m excede maxDistance de ${this.maxDistance}m`);
        return;
      }
      
      // Usar la función utilitaria mejorada
      const { x, y } = this.robotCoordinatesToChart(meters, point.angle);
      // console.log("Cartesian: ",x,y);
     
        const newPoint = this.svg
          .append('circle')
          .attr('cx', x)
          .attr('cy', y)
          .attr('r', 3)
          .attr('fill', 'black');
          
        // Generar ID único para el punto
        const pointId = Date.now() + Math.random();
        this.pointsMap.set(pointId, newPoint);
        
        setTimeout(() => {
          newPoint.remove();
          this.pointsMap.delete(pointId);
        }, this.pointLifetime);
       
    });

    // Vaciar la lista de puntos ya procesados
    this.pointsToPlot = [];
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
   * Creates and initializes the chart with SVG elements.
   * It sets the dimensions of the chart, adds a white background, 
   * draws a border around the content area, and places the robot in the center of the chart.
   */
  createChart() {
    this.svg = d3
      .select('#chart')
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background', '#ffffff'); // Fondo blanco

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

    // Definir los puntos del triángulo
    const triangleSize = 20;
    const cx = this.width / 2;
    const cy = this.height / 2;

    const points = [
        [cx, cy - triangleSize], // Punta arriba
        [cx - triangleSize, cy + triangleSize], // Esquina inferior izquierda
        [cx + triangleSize, cy + triangleSize]  // Esquina inferior derecha
    ].map(p => p.join(',')).join(' ');

    // Dibujar el triángulo en el centro
    this.svg
      .append('polygon')
      .attr('points', points)
      .attr('fill', '#213A7D');

        // Dibujar las circunferencias con los radios dados
        const radii = [0.5, 1.0, 1.5, 2.0]; // Radios en metros (ajustados para maxDistance = 2m)
        radii.forEach(radius => {
            this.svg
              .append('circle')
              .attr('cx', cx)
              .attr('cy', cy)
              .attr('r', radius * this.scaleFactor) // Escalar el radio
              .attr('fill', 'none')
              .attr('stroke', '#213A7D')
              .attr('stroke-width', 2);
        });
}


  /**
   * Clears all stored points on the map by removing their corresponding graphical elements 
   * from the SVG and resetting the points map.
   */
  restartMapping(){
    // Recorrer todos los puntos almacenados en el mapa
    this.pointsMap.forEach((point) => {
      point.remove(); // Eliminar el elemento gráfico del SVG
    });

    // Limpiar el mapa de puntos
    this.pointsMap.clear();
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
      this.width / 2, 
      this.height / 2, 
      true // Invertir Y para SVG
    );
  }


  /**
   * Captures the current SVG map as an image, including a border around it. 
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
  
    const image = new Image();
    image.onload = () => {
      // Dibujar la imagen SVG centrada dentro del borde
      context?.drawImage(image, borderSize, borderSize, this.width, this.height);
      const dataURL = canvas.toDataURL('image/png');
  
      // Descargar la imagen
      const link = document.createElement('a');
      link.href = dataURL;
      link.download = 'map-capture-with-borders.png';
      link.click();
    };
    image.src = `data:image/svg+xml;base64,${btoa(svgString)}`;
  }
  
  

}
