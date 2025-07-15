import { HttpClient, HttpErrorResponse } from '@angular/common/http';
import { Injectable } from '@angular/core';
import { Observable, catchError, throwError, of } from 'rxjs';

@Injectable({
  providedIn: 'root',
})
export class MessagesService {
  //private apiUri = 'http://localhost:1880/message';
  private apiUri = 'http://192.168.1.104:1880/message';
  constructor(private http: HttpClient) {}

  /**
   * Fetches the last message from the backend by sending a GET request to the `/last` endpoint.
   * If an error occurs during the request, it logs the error to the console and returns an 
   * observable with a `null` value to handle the failure gracefully.
   */
  getLastMessage(): Observable<any> {
    return this.http.get<any>(this.apiUri+"/last").pipe(
      catchError((error) => {
        console.error('Error fetching the last message:', error);
        // Devuelve un valor por defecto o un observable vacío
        return of(null); // Puedes devolver un objeto por defecto si lo prefieres
      })
    );
  }
}
