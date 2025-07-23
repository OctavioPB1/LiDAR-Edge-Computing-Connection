import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { MessagesService } from '../../../core/services/messages.service';
import { Subscription, interval, switchMap } from 'rxjs';

export interface Message {
  id?: string;
  tag: string;
  type: 'INFO' | 'WARNING' | 'ERROR';
  message: string;
  timestamp: string;
}

/**
 * @component MonitorComponent
 * @description This component handles the display of monitoring data, such as logs or status updates. 
 * It uses a template and style for the layout and visual presentation. It integrates with other 
 * components for displaying information from sensors or the backend.
 * 
 * The component is self-contained and can be used independently, providing functionality 
 * to show system status, errors, or messages.
 */
@Component({
  selector: 'app-monitor',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './monitor.component.html',
  styleUrl: './monitor.component.scss',
})

export class MonitorComponent {
  isPaused: boolean = false;
  
  messages: any = [];
  private messageTimeouts: Map<string, any> = new Map();

  private subscription!: Subscription;
  constructor(private messageService: MessagesService) {}

  /**
   * Adds a new message to the messages array if the system is not paused. 
   * The message is augmented with a timestamp of when it was received and a unique ID.
   * Each message is automatically removed after 30 seconds.
   * 
   * @param message The message to be added to the messages list.
   */
  addMessage(message: Message) {
    if (!this.isPaused) {
      const timestamp = new Date().toLocaleTimeString();
      if(message !== null) {
        // Generar ID único para el mensaje
        const messageId = Date.now().toString() + Math.random().toString(36).substr(2, 9);
        const messageWithId = { ...message, timestamp, id: messageId };
        
        this.messages.push(messageWithId);
        
        // Programar eliminación automática después de 30 segundos
        const timeoutId = setTimeout(() => {
          this.removeMessage(messageId);
        }, 30000); // 30 segundos
        
        // Guardar referencia del timeout para poder cancelarlo si es necesario
        this.messageTimeouts.set(messageId, timeoutId);
      }
    }
  }

  /**
   * Removes a specific message by its ID.
   * Also clears the associated timeout to prevent memory leaks.
   * 
   * @param messageId The ID of the message to remove.
   */
  removeMessage(messageId: string): void {
    // Encontrar y eliminar el mensaje
    this.messages = this.messages.filter((msg: Message) => msg.id !== messageId);
    
    // Limpiar el timeout asociado
    const timeoutId = this.messageTimeouts.get(messageId);
    if (timeoutId) {
      clearTimeout(timeoutId);
      this.messageTimeouts.delete(messageId);
    }
  }

  /**
   * Initializes the component by subscribing to a service that provides the latest messages.
   * Every second, it fetches the most recent message and adds it to the list of messages.
   * 
   * @note The subscription continues until the component is destroyed or the observable is completed.
   */
  ngOnInit(): void {
    this.subscription = interval(1000)
      .pipe(switchMap(() => this.messageService.getLastMessage()))
      .subscribe({
        next: (message) => {
          this.addMessage(message);
        },
        error: (err) => {
          console.error('Error al obtener el ultimo mensage', err);
        },
      });
  }
    
  /**
   * Cleanup method called when the component is destroyed.
   * Unsubscribes from the message service and clears all pending timeouts.
   */
  ngOnDestroy(): void {
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
    
    // Limpiar todos los timeouts pendientes
    this.messageTimeouts.forEach((timeoutId) => {
      clearTimeout(timeoutId);
    });
    this.messageTimeouts.clear();
  }

  /**
   * Toggles the pause state of the message retrieval process.
   * If `pause` is true, the system pauses adding messages; otherwise, it resumes.
   * 
   * @param pause Boolean indicating whether the process should be paused or not.
   */
  togglePause(pause: boolean): void {
    this.isPaused = pause;
  }
  

  /**
   * Clears all the messages stored in the message list.
   * Also clears all pending timeouts to prevent memory leaks.
   * Resets the array to an empty state.
   */
  clearMessages(): void {
    // Limpiar todos los timeouts pendientes
    this.messageTimeouts.forEach((timeoutId) => {
      clearTimeout(timeoutId);
    });
    this.messageTimeouts.clear();
    
    this.messages = []; // Vacía el array de mensajes
  }  
  
}
