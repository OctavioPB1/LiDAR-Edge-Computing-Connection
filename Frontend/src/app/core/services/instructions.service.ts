import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';
import { catchError, Observable, throwError } from 'rxjs';

@Injectable({
  providedIn: 'root',
})
export class InstructionsService {
  //private apiUri = 'http://localhost:1880/instruction';
  private apiUri = 'http://192.168.1.104:1880/instruction';

  constructor(private http: HttpClient) {}

  /**
   * Sends a POST request to the backend to create a new instruction.
   * If an error occurs during the request, it logs the error to the console and 
   * returns an observable that throws a custom error with a message indicating 
   * the failure to create the instruction.
   */
  createInstruction(inst: any): Observable<any> {
    console.log(inst);
    return this.http.post<any>(this.apiUri, inst).pipe(
      catchError((error) => {
        console.error('Error al crear la instrucción:', error);
        // Puedes lanzar un error personalizado o el error original
        return throwError(
          () => new Error('Error al crear la instrucción. Inténtalo de nuevo.')
        );
      })
    );
  }
}
