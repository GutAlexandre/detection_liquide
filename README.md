class SensorController : Class qui gerer les methodes de detection de liquide, soit readpressure qui lit le capteur de pression, soit par conductance.

void get_message() : Recepetion des messages de type dilbert RS485

void loop() {
  get_message();
  if (millis() - lastSerialCheckTime >= serialCheckInterval) {
    lastSerialCheckTime = millis();
    sensorController.stateUpdate();
    sensorController.readPressure();
  }
}


met a jour le capteur de pression toute les 8ms ce qui permet de priorisé la communication RS485