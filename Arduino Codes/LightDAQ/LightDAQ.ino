//Name:         LightDAQ
//Description:  Light Data Acquisition
//Author:       David Velasquez
//Date:         22/10/2017
//Description: This program communicates with LabVIEW and allows system identification.

//I/O pin definition
#define LDR 0 //Light dependant resistor on pin A0
#define POT 7 //Potentiometer on pin A7
#define PLED 3  //Power LEDs on pin 3

//Constant definitions
const unsigned int numReadings = 2;  //Numero de muestras para promediar (media movil recursiva)
const unsigned long TPRINT = 1000;  //Tiempo de impresion en el monitor serial (debug)

//Variable definitions
double Setpoint = 0, Input = 0; //Set-point fijado desde LabVIEW, medición de la LDR.
//Variables para Media movil recursiva (para filtrar ruido electronico)
unsigned int readings[numReadings] = {0};
unsigned int readIndex = 0;
unsigned int total = 0;
//Para comunicacion con LabVIEW puede comentarlo luego si no usa este software
String writebuffer = "";
String readbuffer = "";
//Tiempo inicial para impresion por serial
unsigned long tini = 0;

//Subrutinas y funciones
unsigned int smooth() { //Rutina de media movil recursiva
  // subtract the last reading
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(LDR);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  return total / numReadings;
}

void setup() {
  //Pin configuration
  pinMode(PLED, OUTPUT);

  //Output cleaning
  digitalWrite(PLED, LOW);

  //Communications
  Serial.begin(9600);
  tini = millis();  //Reset del tiempo inicial
}

void loop() {
  Input = smooth();
    if (Serial.available() > 0) { //Comunicacion con LabVIEW sincronica, comentar todo este if si no tiene LabVIEW
      readbuffer = Serial.readStringUntil('\n');  //Leo hasta el fin de linea el string de LabVIEW
      Setpoint = readbuffer.toInt();  //Convierto o hago un parse a entero de ese String
      readbuffer = "";
      Serial.flush();
      Serial.println(Input);  //Imprimo de vuelta a LabVIEW cuanto esta la medicion para graficarla
    }

  analogWrite(PLED, Setpoint*255.0/1023.0);  //Para identificar la planta, cuando no tiene sintonizado el controlador, debe adquirir los datos de la planta
}
