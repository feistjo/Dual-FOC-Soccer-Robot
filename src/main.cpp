#include <Arduino.h>
#include "SimpleFOC.h"

std::array<BLDCMotor, 2> motors{BLDCMotor(7), BLDCMotor(7)};
// BLDCMotor motor_left{7};
// BLDCMotor motor_right{7};

std::array<BLDCDriver3PWM, 2> drivers{BLDCDriver3PWM(32, 33, 25), BLDCDriver3PWM(26, 27, 14)};
// BLDCDriver3PWM driver_left{32, 33, 25};  // A, B, C for left driver
// BLDCDriver3PWM driver_right{26, 27, 14}; // A, B, C for right driver

float target_position = 0;
// instantiate the commander for testing
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_position, cmd); }
void doLimit(char *cmd)
{
  for (BLDCMotor motor : motors)
    command.scalar(&motor.voltage_limit, cmd);
}
void doVelocity(char *cmd)
{
  for (BLDCMotor motor : motors)
    command.scalar(&motor.velocity_limit, cmd);
}

void setup()
{
  for (int i = 0; i < motors.size(); i++)
  {
    drivers[i].voltage_power_supply = 12;
    drivers[i].voltage_limit = 6;
    drivers[i].init();
    motors[i].linkDriver(&drivers[i]);
    motors[i].voltage_limit = 3;
    motors[i].velocity_limit = 3.14; // rad/s
    motors[i].controller = MotionControlType::angle_openloop;
    motors[i].foc_modulation = FOCModulationType::SpaceVectorPWM; // default is SinePWM
    motors[i].init();
  }

  command.add('T', doTarget, "target angle");
  command.add('L', doLimit, "voltage limit");
  command.add('V', doLimit, "movement velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target position [rad]");
  _delay(1000);
}

void loop()
{
  // open  loop angle movements
  // using motor.voltage_limit and motor.velocity_limit
  for (BLDCMotor motor : motors)
    motor.move(target_position);

  // user communication
  command.run();
}