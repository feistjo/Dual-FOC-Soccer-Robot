#include <Arduino.h>

#include "SimpleFOC.h"

// std::array<BLDCMotor, 2> motors{BLDCMotor(2), BLDCMotor(2)};
std::array<BLDCMotor, 2> motors{BLDCMotor(7), BLDCMotor(7)};
//  BLDCMotor motor_left{7};
//  BLDCMotor motor_right{7};

std::array<BLDCDriver3PWM, 2> drivers{BLDCDriver3PWM(26, 25, 4), BLDCDriver3PWM(16, 17, 21)};
// BLDCDriver3PWM driver_left{32, 33, 25};  // A, B, C for left driver
// BLDCDriver3PWM driver_right{26, 27, 14}; // A, B, C for right driver

std::array<LowsideCurrentSense, 2> current_sensors{LowsideCurrentSense(1100, 34, 39),
                                                   LowsideCurrentSense(1100, 33, 32)};

std::array<MagneticSensorSPI, 2> encoders{MagneticSensorSPI(AS5048_SPI, 23), MagneticSensorSPI(AS5048_SPI, 22)};

std::array<float, 2> phase_resistances{2.0, 1.0};
std::array<float, 2> encoder_offsets{1.0, 0.5};
std::array<Direction, 2> directions{Direction::CCW, Direction::CW};

float target_position = 10;
float target_move = 10;
float velocity_limit = 10;
// instantiate the commander for testing
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_position, cmd); }
void doLimit(char *cmd)
{
    for (int i = 0; i < motors.size(); i++) command.scalar(&motors[i].voltage_limit, cmd);
}
void doVelocity(char *cmd)
{
    for (int i = 0; i < motors.size(); i++) command.scalar(&motors[i].velocity_limit, cmd);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting");
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i].useMonitoring(Serial);
        encoders[i].init();
        motors[i].linkSensor(&encoders[i]);
        drivers[i].voltage_power_supply = 12;
        drivers[i].voltage_limit = 12;
        drivers[i].init();
        motors[i].linkDriver(&drivers[i]);
        current_sensors[i].linkDriver(&drivers[i]);
        current_sensors[i].init();
        // motors[i].linkCurrentSense(&current_sensors[i]);
        //  motors[i].torque_controller = TorqueControlType::dc_current;
        motors[i].voltage_limit = 12;
        motors[i].current_limit = 2;
        motors[i].phase_resistance = phase_resistances[i];
        motors[i].velocity_limit = velocity_limit;  // rad/s
        // motors[i].controller = MotionControlType::velocity_openloop;
        motors[i].torque_controller = TorqueControlType::voltage;
        motors[i].controller = MotionControlType::angle;
        motors[i].foc_modulation = FOCModulationType::SpaceVectorPWM;  // default is SinePWM

        // foc currnet control parameters (stm/esp/due/teensy)
        /* motors[i].PID_current_q.P = 5;
        motors[i].PID_current_q.I = 1000;
        motors[i].PID_current_d.P = 5;
        motors[i].PID_current_d.I = 1000;
        motors[i].LPF_current_q.Tf = 0.002f;  // 1ms default
        motors[i].LPF_current_d.Tf = 0.002f;  // 1ms default */

        motors[i].init();
        // comment out if not needed
        // motors[i].useMonitoring(Serial);
        // motors[i].monitor_variables |= _MON_CURR_D | _MON_CURR_Q;
        motors[i].initFOC(encoder_offsets[i], directions[i]);
    }

    command.add('T', doTarget, "target position");
    command.add('L', doLimit, "voltage limit");
    command.add('V', doLimit, "movement velocity");

    Serial.println("Motor ready!");
    Serial.println("Set target position [rad]");
    _delay(1000);
}

void loop()
{
    static uint32_t last_time = millis();
    if (millis() > last_time + 10000)
    {
        target_position += target_move;
        last_time = millis();
    }
    // open  loop angle movements
    // using motor.voltage_limit and motor.velocity_limit
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i].loopFOC();
        // motors[i].monitor();
        motors[i].move(target_position * (i == 0 ? 1 : 1));
    }

    // user communication
    command.run();
}