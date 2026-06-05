
#include <SimpleFOC.h>

const long BAUD = 115200;

const int MOTOR_1  =  9;
const int MOTOR_2  = 10;
const int MOTOR_3  = 11;
const int MOTOR_EN =  8;

const int SUPPLY_VOLTAGE = 12;
const float VOLTAGE_LIMIT = 4;
const float CURRENT_LIMIT = 1;

float torque_setting = 0;

bool vibrating = false;
float vibrate_magnitude = 0;
float vibrate_center = 0;
int vibrate_phase = 1;
unsigned long last_toggle_us = 0;
const unsigned long VIBRATE_HALF_PERIOD_US = 2000; // 250 Hz

// Initialize the I2C sensor (AS5600)
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_EN);

void setup() {

  Serial.begin(BAUD);

  encoder.init();
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::torque;

  motor.phase_resistance = 2.3;
  motor.KV_rating = 220;
  // motor.axis_inductance.q = 0.01; // ex. 10 mH

  motor.updateVoltageLimit(VOLTAGE_LIMIT);
  motor.updateCurrentLimit(1.2);

  // motor.LPF_velocity.Tf = 0.05;
  // motor.LPF_angle.Tf = 0.005;

  motor.init();
  motor.initFOC();

  Serial.println("Hello");

}

void loop() {
  motor.loopFOC();

  if (vibrating) {
    unsigned long now = micros();
    if (now - last_toggle_us >= VIBRATE_HALF_PERIOD_US) {
      vibrate_phase *= -1;
      last_toggle_us = now;
    }
    motor.move(vibrate_magnitude * vibrate_phase + vibrate_center);
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "READ") {
      Serial.println(motor.shaftAngle());
    } else if (command == "DISABLE") {
      vibrating = false;
      Serial.println("DISABLED");
      motor.disable();
    } else if (command == "ENABLE") {
      Serial.println("ENABLED");
      motor.enable();
    } else if (command.startsWith("VIBRATE")) {
      int space = command.indexOf(' ', 8);
      if (space > 0) {
        vibrate_magnitude = command.substring(8, space).toFloat();
        vibrate_center = command.substring(space + 1).toFloat();
      } else {
        vibrate_magnitude = command.substring(8).toFloat();
        vibrate_center = 0;
      }
      vibrating = true;
      last_toggle_us = micros();
      vibrate_phase = 1;
      Serial.println("VIBRATING " + String(vibrate_magnitude));
    } else {
      torque_setting = command.toFloat();
      vibrating = false;
      motor.move(torque_setting);
      Serial.println("set " + String(torque_setting));
    }
  }

}
