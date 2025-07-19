import { Component, OnInit } from '@angular/core';
import { MappingValueService } from '../../../core/services/mapping-value.service';
import * as d3 from 'd3';
import { polarToCartesianScaled, CartesianCoordinate } from '../../utils/coordinate-transform.util';

@Component({
  selector: 'app-map-heatmap',
  standalone: true,
  imports: [],
  template: `
    <div class="map-container">
      <div class="controls">
        <button (click)="toggleMapping()" [class.active]="mapping">
          {{mapping ? 'Pausar' : 'Iniciar'}} Mapeo
        </button>
        <button (click)="restartMapping()">Limpiar Mapa</button>
        <button (click)="captureMap()">Capturar Imagen</button>
      </div>
      <div id="heatmap-chart"></div>
    </div>
  `,
  styles: [`
    .map-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px;
    }
    .controls {
      margin-bottom: 20px;
      display: flex;
      gap: 10px;
    }
    .controls button {
      padding: 10px 20px;
      border: none;
      border-radius: 5px;
      background: #213A7D;
      color: white;
      cursor: pointer;
    }
    .controls button:hover {
      background: #1a2d63;
    }
    .controls button.active {
      background: #4CAF50;
    }
  `]
})
export class MapHeatmapComponent implements OnInit {
  public mapping: boolean = true;
  private width = 850;
  private height = 850;
  private svg: any;
  private maxDistance = 2;
  private scaleFactor = this.width / (this.maxDistance * 2);
  private pointsToPlot: { distance: number; angle: number }[] = [];

  // Escala de colores para el heatmap
  private colorScale = d3.scaleSequential(d3.interpolateViridis)
    .domain([0, this.maxDistance]);

  constructor(private mappingValueService: MappingValueService) {}

  ngOnInit() {
    this.createChart();
    setInterval(() => this.receivePointsFromBackend(), 1000);
    setInterval(() => this.plotHeatmapPoints(), 50);
  }

  createChart() {
    this.svg = d3
      .select('#heatmap-chart')
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background', '#000011'); // Fondo oscuro para mejor contraste

    // Borde
    this.svg
      .append('rect')
      .attr('x', 0)
      .attr('y', 0)
      .attr('width', this.width)
      .attr('height', this.height)
      .attr('fill', 'none')
      .attr('stroke', '#213A7D')
      .attr('stroke-width', 4);

    // Robot en el centro
    const cx = this.width / 2;
    const cy = this.height / 2;
    const triangleSize = 15;

    const points = [
        [cx, cy - triangleSize],
        [cx - triangleSize, cy + triangleSize],
        [cx + triangleSize, cy + triangleSize]
    ].map(p => p.join(',')).join(' ');

    this.svg
      .append('polygon')
      .attr('points', points)
      .attr('fill', '#ff6b6b')
      .attr('stroke', '#ffffff')
      .attr('stroke-width', 2);

    // Círculos de referencia
    const radii = [0.5, 1.0, 1.5, 2.0];
    radii.forEach((radius, index) => {
      this.svg
        .append('circle')
        .attr('cx', cx)
        .attr('cy', cy)
        .attr('r', radius * this.scaleFactor)
        .attr('fill', 'none')
        .attr('stroke', '#ffffff')
        .attr('stroke-width', 1)
        .attr('opacity', 0.3);
        
      // Etiquetas de distancia
      this.svg
        .append('text')
        .attr('x', cx + radius * this.scaleFactor + 5)
        .attr('y', cy)
        .attr('fill', '#ffffff')
        .attr('font-size', '12px')
        .text(`${radius}m`);
    });
  }

  receivePointsFromBackend(): void {
    this.mappingValueService.getMappingValues().subscribe((data) => {
      if (Array.isArray(data) && data.length > 0) {
        data.forEach((point) => {
          this.pointsToPlot.push(point);
        });
      }
    });
  }

  plotHeatmapPoints(): void {
    if (!this.pointsToPlot.length || !this.mapping) {
      return;
    }

    this.pointsToPlot.forEach((point) => {
      const meters = point.distance / 250; // Tu ajuste actual
      
      if (meters > this.maxDistance) {
        return;
      }

      const { x, y } = this.robotCoordinatesToChart(meters, point.angle);
      
      // Color basado en la distancia
      const color = this.colorScale(meters);
      
      // Tamaño del punto basado en la distancia (más cerca = más grande)
      const pointSize = Math.max(2, 8 - (meters / this.maxDistance) * 6);

      const newPoint = this.svg
        .append('circle')
        .attr('cx', x)
        .attr('cy', y)
        .attr('r', pointSize)
        .attr('fill', color)
        .attr('opacity', 0.8)
        .attr('stroke', '#ffffff')
        .attr('stroke-width', 0.5);

      // Efecto de fade out
      setTimeout(() => {
        newPoint
          .transition()
          .duration(500)
          .attr('opacity', 0)
          .remove();
      }, 2000);
    });

    this.pointsToPlot = [];
  }

  robotCoordinatesToChart(distance: number, angle: number): CartesianCoordinate {
    return polarToCartesianScaled(
      distance, 
      angle, // Sin rotación según tu preferencia
      this.scaleFactor, 
      this.width / 2, 
      this.height / 2, 
      true
    );
  }

  toggleMapping() {
    this.mapping = !this.mapping;
  }

  restartMapping() {
    this.svg.selectAll('circle:not(.reference)').remove();
  }

  captureMap() {
    // Implementación de captura similar a la original
    const svgElement = this.svg.node() as SVGSVGElement;
    const serializer = new XMLSerializer();
    const svgString = serializer.serializeToString(svgElement);
    
    const canvas = document.createElement('canvas');
    canvas.width = this.width;
    canvas.height = this.height;
    const context = canvas.getContext('2d');
    
    const image = new Image();
    image.onload = () => {
      context?.drawImage(image, 0, 0);
      const dataURL = canvas.toDataURL('image/png');
      
      const link = document.createElement('a');
      link.href = dataURL;
      link.download = 'lidar-heatmap.png';
      link.click();
    };
    image.src = `data:image/svg+xml;base64,${btoa(svgString)}`;
  }
} 