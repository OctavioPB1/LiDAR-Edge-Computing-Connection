import { Component, OnInit } from '@angular/core';
import { MappingValueService } from '../../../core/services/mapping-value.service';
import * as d3 from 'd3';
import { polarToCartesianScaled, CartesianCoordinate } from '../../utils/coordinate-transform.util';

@Component({
  selector: 'app-map-radar',
  standalone: true,
  imports: [],
  template: `
    <div class="radar-container">
      <div class="controls">
        <button (click)="toggleMapping()" [class.active]="mapping">
          {{mapping ? 'Pausar' : 'Iniciar'}} Radar
        </button>
        <button (click)="restartMapping()">Limpiar Pantalla</button>
        <button (click)="toggleSweep()" [class.active]="sweepActive">
          {{sweepActive ? 'Parar' : 'Iniciar'}} Barrido
        </button>
      </div>
      <div id="radar-chart"></div>
      <div class="radar-info">
        <div class="distance-legend">
          <div class="legend-item" style="background: #00ff00;">Cerca (0-0.5m)</div>
          <div class="legend-item" style="background: #ffff00;">Medio (0.5-1m)</div>
          <div class="legend-item" style="background: #ff8800;">Lejos (1-1.5m)</div>
          <div class="legend-item" style="background: #ff0000;">Muy lejos (1.5-2m)</div>
        </div>
      </div>
    </div>
  `,
  styles: [`
    .radar-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px;
      background: #001122;
      color: #00ff00;
      font-family: 'Courier New', monospace;
    }
    .controls {
      margin-bottom: 20px;
      display: flex;
      gap: 10px;
    }
    .controls button {
      padding: 10px 20px;
      border: 2px solid #00ff00;
      border-radius: 5px;
      background: transparent;
      color: #00ff00;
      cursor: pointer;
      font-family: 'Courier New', monospace;
    }
    .controls button:hover {
      background: #00ff0033;
    }
    .controls button.active {
      background: #00ff00;
      color: #001122;
    }
    .radar-info {
      margin-top: 20px;
      text-align: center;
    }
    .distance-legend {
      display: flex;
      gap: 15px;
      justify-content: center;
    }
    .legend-item {
      padding: 5px 10px;
      border-radius: 3px;
      font-size: 12px;
      color: #001122;
      font-weight: bold;
    }
  `]
})
export class MapRadarComponent implements OnInit {
  public mapping: boolean = true;
  public sweepActive: boolean = true;
  private width = 850;
  private height = 850;
  private svg: any;
  private maxDistance = 2;
  private scaleFactor = this.width / (this.maxDistance * 2);
  private pointsToPlot: { distance: number; angle: number }[] = [];
  
  // Variables para el barrido
  private sweepAngle = 0;
  private sweepLine: any;
  private sweepGradient: any;

  constructor(private mappingValueService: MappingValueService) {}

  ngOnInit() {
    this.createRadarChart();
    setInterval(() => this.receivePointsFromBackend(), 500);
    setInterval(() => this.plotRadarPoints(), 30);
    setInterval(() => this.updateSweepLine(), 50); // Barrido cada 50ms
  }

  createRadarChart() {
    this.svg = d3
      .select('#radar-chart')
      .append('svg')
      .attr('width', this.width)
      .attr('height', this.height)
      .style('background', '#001122');

    // Definir gradiente para el barrido
    const defs = this.svg.append('defs');
    this.sweepGradient = defs.append('radialGradient')
      .attr('id', 'sweepGradient')
      .attr('cx', '0%')
      .attr('cy', '0%')
      .attr('r', '100%');
    
    this.sweepGradient.append('stop')
      .attr('offset', '0%')
      .attr('stop-color', '#00ff00')
      .attr('stop-opacity', 0.8);
      
    this.sweepGradient.append('stop')
      .attr('offset', '100%')
      .attr('stop-color', '#00ff00')
      .attr('stop-opacity', 0);

    const cx = this.width / 2;
    const cy = this.height / 2;

    // Círculos de rango concéntricos
    const radii = [0.5, 1.0, 1.5, 2.0];
    radii.forEach((radius, index) => {
      this.svg
        .append('circle')
        .attr('cx', cx)
        .attr('cy', cy)
        .attr('r', radius * this.scaleFactor)
        .attr('fill', 'none')
        .attr('stroke', '#00ff00')
        .attr('stroke-width', 1)
        .attr('opacity', 0.4);
        
      // Etiquetas de distancia
      this.svg
        .append('text')
        .attr('x', cx + radius * this.scaleFactor + 5)
        .attr('y', cy - 5)
        .attr('fill', '#00ff00')
        .attr('font-size', '12px')
        .attr('font-family', 'Courier New')
        .text(`${radius}m`);
    });

    // Líneas de ángulo (cada 30 grados)
    for (let angle = 0; angle < 360; angle += 30) {
      const radians = (angle * Math.PI) / 180;
      const x2 = cx + Math.cos(radians) * this.maxDistance * this.scaleFactor;
      const y2 = cy - Math.sin(radians) * this.maxDistance * this.scaleFactor;
      
      this.svg
        .append('line')
        .attr('x1', cx)
        .attr('y1', cy)
        .attr('x2', x2)
        .attr('y2', y2)
        .attr('stroke', '#00ff00')
        .attr('stroke-width', 0.5)
        .attr('opacity', 0.3);
        
      // Etiquetas de ángulo
      const labelRadius = this.maxDistance * this.scaleFactor + 20;
      const labelX = cx + Math.cos(radians) * labelRadius;
      const labelY = cy - Math.sin(radians) * labelRadius;
      
      this.svg
        .append('text')
        .attr('x', labelX)
        .attr('y', labelY)
        .attr('fill', '#00ff00')
        .attr('font-size', '12px')
        .attr('font-family', 'Courier New')
        .attr('text-anchor', 'middle')
        .text(`${angle}°`);
    }

    // Robot en el centro
    this.svg
      .append('circle')
      .attr('cx', cx)
      .attr('cy', cy)
      .attr('r', 8)
      .attr('fill', '#ff6b6b')
      .attr('stroke', '#ffffff')
      .attr('stroke-width', 2);

    // Línea de barrido inicial
    this.sweepLine = this.svg
      .append('line')
      .attr('x1', cx)
      .attr('y1', cy)
      .attr('x2', cx + this.maxDistance * this.scaleFactor)
      .attr('y2', cy)
      .attr('stroke', 'url(#sweepGradient)')
      .attr('stroke-width', 3)
      .attr('opacity', 0.8);
  }

  updateSweepLine() {
    if (!this.sweepActive) return;

    this.sweepAngle += 2; // Velocidad de barrido
    if (this.sweepAngle >= 360) this.sweepAngle = 0;

    const cx = this.width / 2;
    const cy = this.height / 2;
    const radians = (this.sweepAngle * Math.PI) / 180;
    const x2 = cx + Math.cos(radians) * this.maxDistance * this.scaleFactor;
    const y2 = cy - Math.sin(radians) * this.maxDistance * this.scaleFactor;

    this.sweepLine
      .attr('x2', x2)
      .attr('y2', y2);
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

  plotRadarPoints(): void {
    if (!this.pointsToPlot.length || !this.mapping) {
      return;
    }

    this.pointsToPlot.forEach((point) => {
      const meters = point.distance / 250;
      
      if (meters > this.maxDistance) {
        return;
      }

      const { x, y } = this.robotCoordinatesToChart(meters, point.angle);
      
      // Color basado en la distancia
      let color = '#00ff00'; // Verde para cerca
      if (meters > 1.5) color = '#ff0000';      // Rojo para muy lejos
      else if (meters > 1.0) color = '#ff8800'; // Naranja para lejos
      else if (meters > 0.5) color = '#ffff00'; // Amarillo para medio

      // Crear punto con efecto de aparición
      const newPoint = this.svg
        .append('circle')
        .attr('cx', x)
        .attr('cy', y)
        .attr('r', 0)
        .attr('fill', color)
        .attr('opacity', 0)
        .attr('stroke', '#ffffff')
        .attr('stroke-width', 1);

      // Animación de aparición
      newPoint
        .transition()
        .duration(200)
        .attr('r', 4)
        .attr('opacity', 1);

      // Efecto de fade out después de 3 segundos
      setTimeout(() => {
        newPoint
          .transition()
          .duration(1000)
          .attr('opacity', 0)
          .attr('r', 0)
          .remove();
      }, 3000);
    });

    this.pointsToPlot = [];
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

  toggleMapping() {
    this.mapping = !this.mapping;
  }

  toggleSweep() {
    this.sweepActive = !this.sweepActive;
    this.sweepLine.attr('opacity', this.sweepActive ? 0.8 : 0);
  }

  restartMapping() {
    this.svg.selectAll('circle:not(.robot-center)').remove();
  }
} 