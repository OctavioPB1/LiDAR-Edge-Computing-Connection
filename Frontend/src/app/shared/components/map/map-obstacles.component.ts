import { Component, OnInit } from '@angular/core';
import { MappingValueService } from '../../../core/services/mapping-value.service';
import * as d3 from 'd3';
import { polarToCartesianScaled, CartesianCoordinate } from '../../utils/coordinate-transform.util';

interface ObstaclePoint {
  x: number;
  y: number;
  distance: number;
  angle: number;
  timestamp: number;
}

@Component({
  selector: 'app-map-obstacles',
  standalone: true,
  imports: [],
  template: `
    <div class="obstacles-container">
      <div class="controls">
        <button (click)="toggleMapping()" [class.active]="mapping">
          {{mapping ? 'Pausar' : 'Iniciar'}} Detección
        </button>
        <button (click)="restartMapping()">Limpiar Mapa</button>
        <button (click)="toggleObstacleMode()">
          Modo: {{obstacleMode ? 'Contornos' : 'Puntos'}}
        </button>
        <button (click)="captureMap()">Capturar</button>
      </div>
      <div id="obstacles-chart"></div>
      <div class="info-panel">
        <div class="metric">
          <span class="label">Obstáculos detectados:</span>
          <span class="value">{{obstacleCount}}</span>
        </div>
        <div class="metric">
          <span class="label">Puntos activos:</span>
          <span class="value">{{activePoints}}</span>
        </div>
        <div class="metric">
          <span class="label">Distancia promedio:</span>
          <span class="value">{{averageDistance.toFixed(2)}}m</span>
        </div>
      </div>
    </div>
  `,
  styles: [`
    .obstacles-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      font-family: 'Segoe UI', sans-serif;
      border-radius: 10px;
    }
    .controls {
      margin-bottom: 20px;
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }
    .controls button {
      padding: 12px 24px;
      border: none;
      border-radius: 25px;
      background: rgba(255,255,255,0.2);
      color: white;
      cursor: pointer;
      transition: all 0.3s ease;
      backdrop-filter: blur(10px);
    }
    .controls button:hover {
      background: rgba(255,255,255,0.3);
      transform: translateY(-2px);
    }
    .controls button.active {
      background: rgba(76, 175, 80, 0.8);
    }
    .info-panel {
      margin-top: 20px;
      display: flex;
      gap: 30px;
      background: rgba(0,0,0,0.2);
      padding: 15px 30px;
      border-radius: 20px;
      backdrop-filter: blur(10px);
    }
    .metric {
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    .metric .label {
      font-size: 12px;
      opacity: 0.8;
      margin-bottom: 5px;
    }
    .metric .value {
      font-size: 20px;
      font-weight: bold;
      color: #4CAF50;
    }
  `]
})
export class MapObstaclesComponent implements OnInit {
  public mapping: boolean = true;
  public obstacleMode: boolean = true;
  private width = 850;
  private height = 850;
  private svg: any;
  private maxDistance = 2;
  private scaleFactor = this.width / (this.maxDistance * 2);
  private pointsToPlot: { distance: number; angle: number }[] = [];
  
  // Data para obstáculos
  private obstaclePoints: ObstaclePoint[] = [];
  private obstacleGroups: ObstaclePoint[][] = [];
  
  // Métricas para mostrar
  public obstacleCount = 0;
  public activePoints = 0;
  public averageDistance = 0;

  constructor(private mappingValueService: MappingValueService) {}

  ngOnInit() {
    this.createObstacleChart();
    setInterval(() => this.receivePointsFromBackend(), 800);
    setInterval(() => this.plotObstaclePoints(), 100);
    setInterval(() => this.updateMetrics(), 1000);
    setInterval(() => this.cleanOldPoints(), 2000);
  }

  createObstacleChart() {
    this.svg = d3
      .select('#obstacles-chart')
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background', 'linear-gradient(135deg, #1e3c72 0%, #2a5298 100%)')
      .style('border-radius', '15px')
      .style('box-shadow', 'inset 0 0 50px rgba(0,0,0,0.3)');

    const cx = this.width / 2;
    const cy = this.height / 2;

    // Círculos de distancia con efecto glow
    const radii = [0.5, 1.0, 1.5, 2.0];
    radii.forEach((radius, index) => {
      // Círculo principal
      this.svg
        .append('circle')
        .attr('cx', cx)
        .attr('cy', cy)
        .attr('r', radius * this.scaleFactor)
        .attr('fill', 'none')
        .attr('stroke', '#64b5f6')
        .attr('stroke-width', 2)
        .attr('opacity', 0.6)
        .style('filter', 'drop-shadow(0 0 5px #64b5f6)');
        
      // Etiquetas con mejor estilo
      this.svg
        .append('text')
        .attr('x', cx + radius * this.scaleFactor + 10)
        .attr('y', cy - 10)
        .attr('fill', '#ffffff')
        .attr('font-size', '14px')
        .attr('font-weight', 'bold')
        .style('text-shadow', '2px 2px 4px rgba(0,0,0,0.5)')
        .text(`${radius}m`);
    });

    // Líneas de referencia (N, S, E, W)
    const directions = [
      { angle: 0, label: 'E' },
      { angle: 90, label: 'N' },
      { angle: 180, label: 'W' },
      { angle: 270, label: 'S' }
    ];
    
    directions.forEach(dir => {
      const radians = (dir.angle * Math.PI) / 180;
      const x2 = cx + Math.cos(radians) * this.maxDistance * this.scaleFactor;
      const y2 = cy - Math.sin(radians) * this.maxDistance * this.scaleFactor;
      
      this.svg
        .append('line')
        .attr('x1', cx)
        .attr('y1', cy)
        .attr('x2', x2)
        .attr('y2', y2)
        .attr('stroke', '#81c784')
        .attr('stroke-width', 1)
        .attr('opacity', 0.4)
        .attr('stroke-dasharray', '5,5');
        
      // Etiquetas de dirección
      const labelRadius = this.maxDistance * this.scaleFactor + 25;
      const labelX = cx + Math.cos(radians) * labelRadius;
      const labelY = cy - Math.sin(radians) * labelRadius;
      
      this.svg
        .append('text')
        .attr('x', labelX)
        .attr('y', labelY)
        .attr('fill', '#81c784')
        .attr('font-size', '16px')
        .attr('font-weight', 'bold')
        .attr('text-anchor', 'middle')
        .style('text-shadow', '2px 2px 4px rgba(0,0,0,0.5)')
        .text(dir.label);
    });

    // Robot en el centro con efecto más llamativo
    this.svg
      .append('circle')
      .attr('cx', cx)
      .attr('cy', cy)
      .attr('r', 12)
      .attr('fill', '#ff5722')
      .attr('stroke', '#ffffff')
      .attr('stroke-width', 3)
      .style('filter', 'drop-shadow(0 0 10px #ff5722)');

    // Indicador de dirección del robot
    this.svg
      .append('polygon')
      .attr('points', `${cx},${cy-8} ${cx-6},${cy+4} ${cx+6},${cy+4}`)
      .attr('fill', '#ffffff')
      .attr('stroke', '#333')
      .attr('stroke-width', 1);
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

  plotObstaclePoints(): void {
    if (!this.pointsToPlot.length || !this.mapping) {
      return;
    }

    this.pointsToPlot.forEach((point) => {
      const meters = point.distance / 250;
      
      if (meters > this.maxDistance) {
        return;
      }

      const { x, y } = this.robotCoordinatesToChart(meters, point.angle);
      
      // Añadir punto a la colección
      const obstaclePoint: ObstaclePoint = {
        x, y, distance: meters, angle: point.angle,
        timestamp: Date.now()
      };
      
      this.obstaclePoints.push(obstaclePoint);

      if (this.obstacleMode) {
        this.drawObstacleContours();
      } else {
        this.drawIndividualPoint(obstaclePoint);
      }
    });

    this.pointsToPlot = [];
  }

  drawIndividualPoint(point: ObstaclePoint): void {
    // Color basado en distancia
    const hue = (1 - point.distance / this.maxDistance) * 120; // Verde a rojo
    const color = `hsl(${hue}, 80%, 60%)`;
    
    const pointElement = this.svg
      .append('circle')
      .attr('cx', point.x)
      .attr('cy', point.y)
      .attr('r', 0)
      .attr('fill', color)
      .attr('opacity', 0)
      .attr('stroke', '#ffffff')
      .attr('stroke-width', 1)
      .style('filter', `drop-shadow(0 0 5px ${color})`);

    // Animación de aparición
    pointElement
      .transition()
      .duration(300)
      .attr('r', 5)
      .attr('opacity', 0.8);

    // Remover después de 4 segundos
    setTimeout(() => {
      pointElement
        .transition()
        .duration(800)
        .attr('opacity', 0)
        .attr('r', 0)
        .remove();
    }, 4000);
  }

  drawObstacleContours(): void {
    // Limpiar contornos anteriores
    this.svg.selectAll('.obstacle-contour').remove();
    
    // Agrupar puntos cercanos
    this.groupNearbyPoints();
    
    // Dibujar contornos para cada grupo
    this.obstacleGroups.forEach((group, index) => {
      if (group.length < 3) return; // Necesitamos al menos 3 puntos
      
      // Crear path para el contorno
      const line = d3.line<ObstaclePoint>()
        .x(d => d.x)
        .y(d => d.y)
        .curve(d3.curveCardinalClosed);
        
      const hue = (index * 60) % 360; // Diferentes colores para cada obstáculo
      const color = `hsl(${hue}, 70%, 50%)`;
      
      this.svg
        .append('path')
        .datum(group)
        .attr('class', 'obstacle-contour')
        .attr('d', line)
        .attr('fill', color)
        .attr('opacity', 0.3)
        .attr('stroke', color)
        .attr('stroke-width', 2)
        .style('filter', `drop-shadow(0 0 8px ${color})`);
    });
  }

  groupNearbyPoints(): void {
    this.obstacleGroups = [];
    const maxDistance = 50; // Distancia máxima para agrupar puntos
    const processed = new Set<number>();
    
    this.obstaclePoints.forEach((point, index) => {
      if (processed.has(index)) return;
      
      const group: ObstaclePoint[] = [point];
      processed.add(index);
      
      // Buscar puntos cercanos
      this.obstaclePoints.forEach((otherPoint, otherIndex) => {
        if (processed.has(otherIndex)) return;
        
        const distance = Math.sqrt(
          Math.pow(point.x - otherPoint.x, 2) + 
          Math.pow(point.y - otherPoint.y, 2)
        );
        
        if (distance < maxDistance) {
          group.push(otherPoint);
          processed.add(otherIndex);
        }
      });
      
      if (group.length >= 3) {
        this.obstacleGroups.push(group);
      }
    });
  }

  robotCoordinatesToChart(distance: number, angle: number): CartesianCoordinate {
    return polarToCartesianScaled(
      distance, 
      angle,
      this.scaleFactor, 
      this.width / 2, 
      this.height / 2, 
      true
    );
  }

  updateMetrics(): void {
    this.obstacleCount = this.obstacleGroups.length;
    this.activePoints = this.obstaclePoints.length;
    
    if (this.obstaclePoints.length > 0) {
      const totalDistance = this.obstaclePoints.reduce((sum, p) => sum + p.distance, 0);
      this.averageDistance = totalDistance / this.obstaclePoints.length;
    }
  }

  cleanOldPoints(): void {
    const now = Date.now();
    const maxAge = 5000; // 5 segundos
    
    this.obstaclePoints = this.obstaclePoints.filter(
      point => now - point.timestamp < maxAge
    );
  }

  toggleMapping() {
    this.mapping = !this.mapping;
  }

  toggleObstacleMode() {
    this.obstacleMode = !this.obstacleMode;
    this.svg.selectAll('.obstacle-contour').remove();
  }

  restartMapping() {
    this.obstaclePoints = [];
    this.obstacleGroups = [];
    this.svg.selectAll('circle:not(.robot-center)').remove();
    this.svg.selectAll('.obstacle-contour').remove();
  }

  captureMap() {
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
      link.download = 'obstacle-map.png';
      link.click();
    };
    image.src = `data:image/svg+xml;base64,${btoa(svgString)}`;
  }
} 