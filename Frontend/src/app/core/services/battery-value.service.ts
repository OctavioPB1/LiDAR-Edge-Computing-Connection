import { Injectable } from '@angular/core';
import { catchError, Observable, throwError, of } from 'rxjs';
import { HttpClient } from '@angular/common/http';

/**
 * A service responsible for retrieving the battery level from the backend.
 * It fetches the last recorded battery level from the provided API.
 */
@Injectable({
  providedIn: 'root'
})
export class BatteryValueService {
  // The API endpoint for fetching the battery level data
  private apiUri = 'http://localhost:1880/battery-level';

  constructor(private http: HttpClient) { }

  /**
   * Retrieves the latest battery level from the backend.
   * This method sends a GET request to the API and returns the battery level data as an observable.
   * If there is an error while fetching the data, it logs the error and returns a default value (null).
   */
  getBatteryValue(): Observable<any> {
    return this.http.get<any>(this.apiUri + "/last").pipe(catchError((error) => {
      console.error('Error fetching the last battery value:', error);
      // Devuelve un valor por defecto o un observable vacío
      return of(null); // Puedes devolver un objeto por defecto si lo prefieres
    }));
  }
}
