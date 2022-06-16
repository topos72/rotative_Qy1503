#include "Service.h"
#define LOG_PERIOD 500 //intervallo di campionamento dovrebbe essere inferiore ad un giro completo
#define soglia 7
#define INERZIA  7E-4  // momento di inerzia da modificare
boolean ORARIO = false;  // ORARIO =1 verso di rotazione

float Ang_0 = 0;  //default angolo iniziale
boolean RUN = false;
unsigned long previousMillis;  //tempo
float prec_reading ;   //angolo

const int Num = 70;  // problemi di memoria dinamica
float Angle_gradi[Num];
float Angle[Num];
float tempo[Num];
float W_angle[Num];
int conta_giri = 0;
int conta_Num = 0;

void setup()
{
  //setup our pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  //give some default values
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(CS_PIN, HIGH);
  // inizializzo il vettore W_angle[Num]
  for (int t = 0; t < Num; t++) {
    tempo[t]   = 0.0;    Angle[t]  = 0.0 ;    W_angle[t] = 0.0 ;
  }
  Serial.begin(115200);
  delay(100);
  Ang_0 = readPosition();
  // Serial.println(Ang_0);
  float theta_0 = Ang_0 * 3.1416 * (1) / 180.0 ;
  Serial.println("Inizio acquisizione: il sensore resta in attesa del movimento angolare\n fino al superamento della soglia");
 Serial.print("soglia di attivazione in gradi "); Serial.println(soglia);
 Serial.print("intervallo acquisizione dati [ms] "); Serial.println(LOG_PERIOD);
  Serial.print("inerzia  "); Serial.println(INERZIA,5);
  
  if (DEBUG) Serial.println("giri; time[sec];  Angle[rad]; W[rad/s]; Kintetica_rot;"); //elemento separatore ;
  delay(500); // pausa per stabilizzare il sistema di acquisizione dati
  previousMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  float reading = readPosition();
  // codice in loopn RUN =true solo dopo i
   if (DEBUG) Serial.println(reading);
  if (!RUN) {
    F_giro(reading, Ang_0);
  }
  while (RUN) {
    currentMillis = millis();
    //Time tempo di campionamento, acquisisce solo se sono trascorsi piu di LOG_PERIOD millisecondi
    if ((currentMillis - previousMillis) > LOG_PERIOD) {
      if (DEBUG)  Serial.println(" sono trascorsi (LOG_PERIOD) millisecondi ");
      F_giro_B(reading, prec_reading , conta_Num);
    }
    reading = readPosition();
    if (conta_Num > Num - 1) {
      Serial.println("terminata l'acquisizione °°°time°°°angolo°°°w°°°");
      Serial.println("giri; time[sec]; Angle[gradi]; Angle[rad]; W[rad/s]; Kintetica_rot;"); //elemento separatore 
      for (int n = 0; n < Num; n++) {
        Stamp(n, 'K');
        delay(50);
      }
      while (1);  // si ferma l'acquisizione
    }
  }
}

// funzione acuisizione dati 
void F_giro_B(float th_B, float angle_0, int count) {
  if (th_B != -1 && ORARIO) {
    tempo[count] = millis() / 1000.0  - tempo[0] ; //secondi
    if ((th_B - prec_reading) < - soglia ) {
      conta_giri =  conta_giri + 1 ;
      if (DEBUG) Serial.println("---------giro successivo --------");
    }
    float transit = (360 * conta_giri + th_B);
    Angle_gradi[count] = transit;
    float theta = (transit) * 3.1416 / 180.0;  //converte in radianti
    Angle[count] = theta ;
    if (count == 1) {
      W_angle[count] = ( ( Angle[count] - Angle[count - 1] ) / tempo[count] );
    } else {
      W_angle[count] = ( (( Angle[count] - Angle[count - 1] ) / (tempo[count] - tempo[count - 1])) );
    }
    //  Stamp(count, 'K');
    conta_Num = conta_Num + 1;
    previousMillis = millis();
    prec_reading = th_B ;
  } else {
    if (DEBUG) Serial.println(" §§§ la chiamata alla funzione è inutile");
  }
}



//PRIMO giro: colleziona il primo angolo che si discosta dall'angolo iniziale
//di un valore superiore alla soglia , verifica che il moto sia in senso orario
// invia in stampa i vettori con indice conta_Num = 0
void F_giro(float th, float angle_0) {
 
  //  Serial.println(angle_0);
  float t_0 = millis();
  if ((abs(th - angle_0) != 0) &&  ( th != -1)  && (abs(th - angle_0) > soglia)) {
     if (DEBUG) Serial.print("inizia l'acquisizione, primo giro  ");
    if ((th - angle_0) > 0) {
      if (DEBUG) Serial.print("verso di rotazione orario ");
      ORARIO = true;
    } else {
      if (DEBUG) Serial.print("verso di rotazione antiorario ");
      ORARIO = false;
    }
    // tempo[0]   =  0.0; //secondi
    tempo[0]   = (millis() / 1000.0); //secondi
    Angle_gradi[0] = th;
    float theta = (th) * 3.1416 * (1) / 180.0;  //radianti
    Angle[0] = theta ; //  angolo in radianti
    W_angle[0] = 0.0 ;    //  radianti/sec
    Stamp(0, 'K') ;
    conta_Num = 1;
    RUN = true ;
    if (DEBUG) Serial.println("uscita F_giro: valori assegnatia a RUN=TRUE, prec_reading, previousMillis");
    prec_reading  = th;
    previousMillis = millis() ;
  }
}
// funzione stampa risultati acuisizione
void Stamp(int nn, char SET) {
  float K_energy = 0.0;
  Serial.print("##  ");
  Serial.print( nn );
  Serial.print("; ");
  if (nn == 0) {
    Serial.print(0.0, 5);
  } else {
    Serial.print(tempo[nn], 5);
  }
  Serial.print("; ");
  Serial.print(Angle_gradi[nn]);
  Serial.print("; ");
  Serial.print( Angle[nn] );
  Serial.print("; ");
  //Stampa la velocità angolare
  Serial.print(W_angle[nn], 4);
  if (SET == 'K') {
    //Serial.println(SET);
    K_energy = W_angle[nn] * W_angle[nn] * 0.5 * 0.00072;
    Serial.print("; ");
    Serial.print(K_energy, 8);
    Serial.print(";\n");
    delay(50);
  } else {
    Serial.print("\t ;");
    Serial.print("\n");
    delay(50);
  }
  return (1);
}

//Stampa riga time angle W_angle
int Stampa_init(float reading_s, int indice) {
  float temp_second = millis() / 1000.0;
  Serial.print("°°°time°°°angolo°°°w°°°");
  Serial.print(temp_second);
  Serial.print("; ");
  Serial.print( reading_s );
  Serial.print("; ");
  //Stampa la velocità angolare
  Serial.print(W_angle[indice]);
  Serial.print("\n");
  return (1);
}

//read the current angular position
float readPosition()
{
  // Read the same position data twice to check for errors
  unsigned long sample1 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);
  unsigned long sample2 = shiftIn(DATA_PIN, CLOCK_PIN, CS_PIN, BIT_COUNT);
  delayMicroseconds(20); // Clock must be high for 20 microseconds before a new sample can be taken
  if (sample1 != sample2) return -1.0;

  return ((sample1 & 0x0FFF) * 360UL) / 1024.0;         //  RANGE 0-360
  // return ((sample1 & 0x0FFF) * 360UL) / 2048.0;           // RANGE 0-180
  //return ((sample1 & 0x0FFF) * 360UL) / 4096.0;        //  RANGE  0-90
  //return ((sample1 & 0x0FFF) * 360UL) / 512.0;
}

//read in a byte of data from the digital input of the board.
unsigned long shiftIn(const int data_pin, const int clock_pin, const int cs_pin, const int bit_count)
{
  unsigned long data = 0;

  digitalWrite(cs_pin, LOW);
  for (int i = 0; i < bit_count; i++)
  {
    data <<= 1;
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    data |= digitalRead(data_pin);

  }
  //Serial.println(data);   //valore
  digitalWrite(cs_pin, HIGH);
  return data;
}
