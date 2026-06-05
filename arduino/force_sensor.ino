
const long BAUD = 115200;
const int SENSOR_PIN = A0;

void setup() {
  Serial.begin(BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  // TODO: add a wake-up message for the lerobot SOSensorArm to receive when 
  // it connects to this device over serial
}

void loop() {
if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any whitespace or \r

    if (command == "STATUS") {
      Serial.println("OK");
    } 
    else if (command == "READ") {
      int force_reading = analogRead(SENSOR_PIN);
      Serial.println(1023 - force_reading);
    } 
    else {
      Serial.print("ERROR: UNKNOWN_COMMAND: ");
      Serial.println(command);
    }
  }
}