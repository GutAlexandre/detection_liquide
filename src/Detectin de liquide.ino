#include <Adafruit_LPS35HW.h>

const int sender = A0;
const int highThreshold = 30;
const int lowThreshold = -30;
const int time_in_ms = 10000;
bool state = false;
float offset = 0.0;
float conductance1;
float temperature = 0.0;
float pressure = 0.0;

#define LPS33_ADDRESS 0x5D
#define CTRL_REG1 0x10
#define PRESS_OUT_XL 0x28
#define PRESS_OUT_L 0x29
#define PRESS_OUT_H 0x2A

class SensorController {
public:
  SensorController(int analogPin, int highThreshold, int lowThreshold, int time_in_ms)
    : analogPin(analogPin), highThreshold(highThreshold), lowThreshold(lowThreshold), time_in_ms(time_in_ms) {}

  void begin() {
    Serial.begin(115200);  // Utilisez le port série matériel (RX/TX)
    Wire.begin();
    Wire.beginTransmission(LPS33_ADDRESS);
    Wire.write(CTRL_REG1);
    Wire.write(0x90);
    Wire.endTransmission();
  }

  void calculateOffset() {
    Serial.print("Calculating offset...");
    unsigned long startTime = millis();
    unsigned long lastPrintTime = startTime;
    float sum = 0.0;
    int count = 0;

    while (millis() - startTime <= time_in_ms) {
      sum += analogRead(analogPin);
      count++;
      if (millis() - lastPrintTime >= 500) {
        Serial.print('.');
        lastPrintTime = millis();
      }
    }

    offset = sum / count;
    Serial.println("End!");
  }

  void stateUpdate() {
    float newConductance = analogRead(analogPin) - offset;
    if (newConductance > highThreshold && !state) {
      state = true;
    } else if (newConductance < lowThreshold && state) {
      state = false;
    }
  }

  float readPressure() {
    Wire.beginTransmission(LPS33_ADDRESS);
    Wire.write(PRESS_OUT_XL | (1 << 7));
    Wire.endTransmission(false);
    Wire.requestFrom(LPS33_ADDRESS, 3);

    if (Wire.available() >= 3) {
      uint8_t pxl = Wire.read();
      uint8_t pl = Wire.read();
      uint8_t ph = Wire.read();

      int32_t pressure_raw = (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
      pressure = (float)pressure_raw / 4096.0;
    }
    return pressure;
  }

private:
  const int analogPin;
  const int highThreshold;
  const int lowThreshold;
  const int time_in_ms;
};

SensorController sensorController(sender, highThreshold, lowThreshold, time_in_ms);
int DE_Pin = 4;
int RE_Pin = 5;
char receivedMessage[64];
int receivedMessageIndex = 0;

void send_data(const String data, const String answer) {
  data.replace("\r", "");
  data.replace("\n", "");
  Serial.println(data + answer);
}

void rebootArduino() {
  asm volatile("  jmp 0");
}

void get_message() {
  while (Serial.available()) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      receivedMessage[receivedMessageIndex] = '\0'; // Null-terminate the string
      receivedMessageIndex = 0;
      if (strstr(receivedMessage, "#7V") != NULL) {
        send_data(receivedMessage, " ARDU_RS485_07-09-2023");
      } else if (strstr(receivedMessage, "#7CO") != NULL) {
        send_data(receivedMessage, " progress");
        sensorController.calculateOffset();
      } else if (strstr(receivedMessage, "#7O") != NULL) {
        send_data(receivedMessage, String(offset).c_str());
      } else if (strstr(receivedMessage, "#7C") != NULL) {
        send_data(receivedMessage, String(conductance1).c_str());
      } else if (strstr(receivedMessage, "#7S") != NULL) {
        send_data(receivedMessage, String(state).c_str());
      } else if (strstr(receivedMessage, "#7RS") != NULL) {
        send_data(receivedMessage, " progress");
        rebootArduino();
      } else if (strstr(receivedMessage, "#7P") != NULL) {
        float p = sensorController.readPressure();
        char pressureStr[10];
        dtostrf(p, 4, 2, pressureStr);
        send_data(receivedMessage, String(pressure));
      } else {
        send_data(receivedMessage, "?");
      }
    } else {
      receivedMessage[receivedMessageIndex++] = receivedChar;
    }
  }
}

unsigned long lastSerialCheckTime = 0;
const unsigned long serialCheckInterval = 8;

void setup() {
  pinMode(DE_Pin, OUTPUT);
  pinMode(RE_Pin, OUTPUT);
  digitalWrite(DE_Pin, 0);
  digitalWrite(RE_Pin, 1);
  sensorController.begin();
  Serial.println("Begin");
}

void loop() {
  get_message();
  if (millis() - lastSerialCheckTime >= serialCheckInterval) {
    lastSerialCheckTime = millis();
    sensorController.stateUpdate();
    sensorController.readPressure();
  }
}
