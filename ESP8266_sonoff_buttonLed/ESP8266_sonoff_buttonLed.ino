const byte pinLed = 13;
const byte pinPulsador = 0;
 
// Estado LED
boolean estadoLed = true;
 
void setup() {
  // Modo de los pines
  pinMode(pinLed, OUTPUT);
  pinMode(pinPulsador, INPUT);
}
 
void loop() {
  // Comprobamos que esté apretado el pulsador
  if (!digitalRead(pinPulsador)) {
    // Cambiamos el estado
    estadoLed = !estadoLed;
    delay(500);
  }
 
  digitalWrite(pinLed, estadoLed);
}
