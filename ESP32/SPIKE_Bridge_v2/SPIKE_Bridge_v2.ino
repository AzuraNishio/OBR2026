#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
HardwareSerial UART1(1);

int rear_button_pin_L = 5;
int rear_button_pin_R = 18;

// ====================================================================================
// Uart protocol stuff
// ====================================================================================
byte ALT_FLAG  = 0xA0;
byte STOP_FLAG = 0xA5;
byte ALT_ALT_FLAG    = 0xA1;
byte ALT_STOP_FLAG   = 0xA6;

int pointer = 0;
int packetBufferIndex = 0;

byte packetBuffer[8][256];

bool is_alt = false;
bool is_packet_new = false;
void send_packet(byte packet[], int len) {
    for (int i = 0; i < len; i++) {
        byte b = packet[i];

        if (b == ALT_FLAG) {
            UART1.write(ALT_FLAG);
            UART1.write(ALT_ALT_FLAG);
        }
        else if (b == STOP_FLAG) {
            UART1.write(ALT_FLAG);
            UART1.write(ALT_STOP_FLAG);
        }
        else {
            UART1.write(b);
        }
    }
    UART1.write(STOP_FLAG);

}

void _add_byte_to_buffer(byte b) {
    if (pointer >= 256)
        return;

    packetBuffer[packetBufferIndex][pointer] = b;
    pointer++;
}

byte unalt(byte b) {
    if (b == ALT_ALT_FLAG)
        return ALT_FLAG;

    if (b == ALT_STOP_FLAG)
        return STOP_FLAG;

    return b;
}


void tick_listener() {
    while (UART1.available()) {
        byte b = UART1.read();

        if (b == STOP_FLAG) {
            pointer = 0;
            packetBufferIndex = (packetBufferIndex + 1) % 8;
            is_packet_new = true;
        }
        else if (is_alt) {
            _add_byte_to_buffer(unalt(b));
            is_alt = false;
        }
        else if (b == ALT_FLAG) {
            is_alt = true;
        }
        else {
            _add_byte_to_buffer(b);
        }
    }
}

// ====================================================================================
// Sensor working stuff
// ====================================================================================

void startup_distance_sensor(){
  
  while (true) {
    rgb_log(1, 1, 0);
    Wire.begin();
    sensor.setTimeout(500);
    delay(100);


    if (!sensor.init()) {
      Serial.println("VL53L1X INIT FAILED, RETRYING");
      for (int i = 0; i < 6; i++){
        rgb_log(1, 0, 0);
        delay(50);
        rgb_log(0, 0, 0);
        delay(50);
      }
    } else {
      break;
    }
  }

  // Sensor configuration
  rgb_log(1, 1, 0);
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50 ms
  Serial.println("VL53L1X succesfully initialized!");
  sensor.startContinuous(50);

  rgb_log(0, 1, 0);
}

void restart_distance_sensor(){
  sensor.stopContinuous();
  Wire.end();
  startup_distance_sensor();
}



int ledR = 15;
int ledG = 2;
int ledB = 4;

void setup() {


  // Startup logging
  Serial.begin(115200);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(rear_button_pin_L, INPUT);
  pinMode(rear_button_pin_R, INPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);
  Serial.println("ESP32 Hub alive!");
  Serial.println("===============[ Info ]===============");
  Serial.println(ESP.getChipModel());
  Serial.println(ESP.getChipRevision());
  Serial.println(ESP.getChipCores());
  Serial.println("======================================");

  // Hub bridge startup
  UART1.begin(115200, SERIAL_8N1, 16, 17);
  startup_distance_sensor();

}

void rgb_log(float r, float g, float b){
  analogWrite(ledR, 254 * r);
  analogWrite(ledG, 254 * g);
  analogWrite(ledB, 254 * b);
}


void rgb_sine_log(float r, float g, float b, float f){
  float s = (sin(f * millis() * 0.001 * 3.1415) + 1) * 0.5;
  rgb_log(r * s, g * s, b * s);
}

void rgb_sine_log_delay(float r, float g, float b, float f, int dd){
  for (int i = 0; i < dd; i+= 2){
    rgb_sine_log(r, g, b, f);
    delay(2);
  }
}




void loop() {
    static uint16_t last_valid_distance = 0;

    uint16_t distance = sensor.read();

    byte packet[4];
    float r = 0;
    float b = 0;
    byte rear_button_L = 0x00;
    byte rear_button_R = 0x00;
\
    if (digitalRead(rear_button_pin_L) == LOW){
      Serial.print("L-on  "); 
      rear_button_L = 0x01; 
      r = 0.3;
    } else {
      Serial.print("L-off ");
    }
    if (digitalRead(rear_button_pin_R) == LOW){
      Serial.println("R-on"); 
      rear_button_R = 0x01; 
      b = 0.3;
    } else {
      Serial.println("R-off");
    }

    if (sensor.timeoutOccurred()) {
        // Keep the previous valid distance
        packet[0] = lowByte(last_valid_distance);
        packet[1] = highByte(last_valid_distance);
        packet[2] = 0x01; // TIMEOUT
        packet[3] = rear_button_L;
        packet[4] = rear_button_R;

        Serial.println("TIMEOUT");

        send_packet(packet, 5);
        restart_distance_sensor();
    }
    else {
        last_valid_distance = distance;

        packet[0] = lowByte(distance);
        packet[1] = highByte(distance);
        packet[2] = 0x00; // VALID
        packet[3] = rear_button_L;
        packet[4] = rear_button_R;

        //Serial.println(distance);

        send_packet(packet, 5);
    }
    //rgb.log(float(int(packetBuffer[(packetBufferIndex + 7) % 8][0])) / 254, float(int(packetBuffer[(packetBufferIndex + 7) % 8][1])) / 254, float(int(packetBuffer[(packetBufferIndex + 7) % 8][2])) / 254)
    
    rgb_sine_log_delay(r, 0.5, b, 1, 100);
    tick_listener();
    if (is_packet_new){
      is_packet_new = false;
      send_packet(packetBuffer[(packetBufferIndex + 7) % 8], 10);
      rgb_log(0, 0, 1);
      delay(1000);
    }
}
