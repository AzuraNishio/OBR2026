#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
HardwareSerial UART2(2);








void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1000);

  Serial.println("Mewo! Alive!");

  UART2.begin(115200, SERIAL_8N1, 16, 17);


    // I2C
  Wire.begin();
    // Initialize sensor
  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.print("VL53L1X INIT FAILED");
    delay(1000);
  }

    // Distance mode
  sensor.setDistanceMode(VL53L1X::Long);

  // Timing budget
  sensor.setMeasurementTimingBudget(50000); // 50 ms

  // Start continuous measurements
  sensor.startContinuous(50);

}

void loop() {
  uint16_t distance = sensor.read();
  if (millis() % 1000 < 500){
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }
  if (sensor.timeoutOccurred()) {
    UART2.println("TIMEOUT");
    Serial.println("TIMEOUT");
  } else {
    UART2.println(distance);
    Serial.println(distance);
  }

  delay(100);

}
