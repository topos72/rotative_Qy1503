#include "Service.h"
/*

void F_giro(float th, float angle_0) {  //giro iniziale
  if ((abs(th - angle_0) != 0) &&  ( th != -1)  && (abs(th - angle_0) > soglia)) {
   ORARIO = Rotazione(th, angle_0);  // restituisce il senso di rotazione ORario 1 o antiorario 0
    Serial.print("### senso di rotazione ORARIO 1 (ANTIORARIO 0) :  ");
    Serial.print(th);
    Serial.print("; ");
     Serial.print("0 ;  ");
    //colleziona il primo angolo che si discosta dall'angolo iniziale di un valore superiore alla soglia conta_Num = 0
    tempo[0]   = millis() / 1000.0 - 0.0; //secondi
    float theta = (th) * 3.1416 * (1) / 180.0;  //radianti
    Angle[0] = theta ; //angolo in radianti
    W_angle[0] = 0.0 ;    //radianti/sec
    //  Serial.println("## ciclo  iniziale  n.  ");
    // Serial.print(conta_Num);
    // Serial.print(";");
    Serial.print( tempo[conta_Num] / 1000.0 );
    Serial.print("; ");
    Serial.print( Angle[conta_Num] );
    Serial.print("; ");
    Serial.print( W_angle[conta_Num] );
    Serial.print("; ");
    Serial.print( W_angle[conta_Num] );
    Serial.print("\n");
    conta_Num = 1;
    RUN = true;
    previousMillis = millis();
    prec_reading  = th;
  }
}

bool Rotazione(float Angle, float a_0 ) {
  boolean Rot;
  // Serial.println("chiamata a rotazione");
  if ((Angle - a_0) > 0 ) {
    // Serial.print(Angle - Ang_0);
    // Serial.println("     orario");
    Rot = true;
  }
  else {
    // Serial.print(Angle - Ang_0);
    // Serial.println("     anti_orario");
    Rot = false;
    //  Serial.println(ORARIO);
    // Serial.println(Angle);
  }
  return (Rot);
}
*/
