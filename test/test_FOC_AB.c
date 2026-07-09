
#include <Arduino.h>
#include <SimpleFOC.h>

#define POWEREN_PIN 7

// sensor instance
MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);

TwoWire I2C_A = TwoWire(1);
TwoWire I2C_B = TwoWire(0);

// BLDC motor & driver instance
BLDCMotor motor_A = BLDCMotor(7);
BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 34, 33);

BLDCMotor motor_B = BLDCMotor(7);
BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10);

unsigned long lastStatusMs = 0;

void handleMotorCommand(String token)
{
    token.trim();
    token.toUpperCase();
    if (token.length() < 2)
    {
        return;
    }

    const char motorId = token.charAt(0);
    const float target = token.substring(1).toFloat();

    if (motorId == 'A')
    {
        motor_A.target = target;
        Serial.print("Motor A target set to ");
        Serial.print(target);
        Serial.println(" rad/s");
    }
    else if (motorId == 'B')
    {
        motor_B.target = target;
        Serial.print("Motor B target set to ");
        Serial.print(target);
        Serial.println(" rad/s");
    }
    else
    {
        Serial.print("Unknown command: ");
        Serial.println(token);
    }
}

void handleSerialCommands()
{
    if (!Serial.available())
    {
        return;
    }

    String line = Serial.readStringUntil('\n');
    line.trim();
    while (line.length() > 0)
    {
        int space = line.indexOf(' ');
        String token = space >= 0 ? line.substring(0, space) : line;
        handleMotorCommand(token);

        if (space < 0)
        {
            break;
        }
        line = line.substring(space + 1);
        line.trim();
    }
}

void printStatus()
{
    if (millis() - lastStatusMs < 1000)
    {
        return;
    }
    lastStatusMs = millis();

    Serial.print("A target=");
    Serial.print(motor_A.target);
    Serial.print(" vel=");
    Serial.print(motor_A.shaft_velocity);
    Serial.print(" angle=");
    Serial.print(motor_A.shaft_angle);
    Serial.print(" | B target=");
    Serial.print(motor_B.target);
    Serial.print(" vel=");
    Serial.print(motor_B.shaft_velocity);
    Serial.print(" angle=");
    Serial.println(motor_B.shaft_angle);
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(20);
    pinMode(POWEREN_PIN, OUTPUT);
    digitalWrite(POWEREN_PIN, HIGH);
    delay(500);
    Serial.println("Power enable pin 7 set HIGH.");

    // initialise magnetic sensor hardware
    I2C_A.begin(37, 36, 400000UL);
    I2C_B.begin(8, 9, 400000UL);
    sensor_A.init(&I2C_A);
    sensor_B.init(&I2C_B);

    // link the motor to the sensor
    motor_A.linkSensor(&sensor_A);
    motor_B.linkSensor(&sensor_B);

    // driver config
    // power supply voltage [V]
    driver_A.voltage_power_supply = 8.4;
    driver_B.voltage_power_supply = 8.4;
    driver_A.init();
    driver_B.init();
    // link the motor and the driver
    motor_A.linkDriver(&driver_A);
    motor_B.linkDriver(&driver_B);
    // motor config
    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.modulation_centered = 1.0;
    motor_A.modulation_centered = 1.0;
    // set motion control loop to be used
    motor_A.controller = MotionControlType::velocity;
    motor_B.controller = MotionControlType::velocity;

    // velocity PI controller parameters
    motor_A.PID_velocity.P = 0.09f;
    motor_A.PID_velocity.I = 3;
    motor_A.PID_velocity.D = 0;
    motor_A.LPF_velocity.Tf = 0.02f;
    motor_A.PID_velocity.output_ramp = 1000;
    motor_A.voltage_limit = 1;

    motor_B.PID_velocity.P = 0.09f;
    motor_B.PID_velocity.I = 3;
    motor_B.PID_velocity.D = 0;
    motor_B.LPF_velocity.Tf = 0.02f;
    motor_B.PID_velocity.output_ramp = 1000;
    motor_B.voltage_limit = 1;

    // use monitoring with serial
    // comment out if not needed
    motor_A.useMonitoring(Serial);
    motor_B.useMonitoring(Serial);
    // initialize motor
    motor_A.init();
    motor_B.init();
    // align sensor and start FOC
    motor_A.initFOC();
    motor_B.initFOC();

    Serial.println(F("Motor ready."));
    Serial.println(F("Use commands: A9, B-5, A0, B0, or A9 B-5."));
    _delay(1000);
}

void loop()
{
    motor_A.loopFOC();
    motor_B.loopFOC();
    motor_A.move();
    motor_B.move();
    handleSerialCommands();
    printStatus();
}
