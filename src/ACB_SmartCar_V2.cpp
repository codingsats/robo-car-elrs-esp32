#include "ACB_SmartCar_V2.h"
#include <Arduino.h>

ACB_SmartCar_V2::ACB_SmartCar_V2()
    : Serial_2(&Serial2)
{
}

void ACB_SmartCar_V2::Init()
{
    Serial_2->begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    motorControl(1, 0);
    motorControl(2, 0);
    motorControl(3, 0);
    motorControl(4, 0);
}

void ACB_SmartCar_V2::motorControl(uint8_t motor, int speed)
{
    if (Serial_2 == nullptr) return;

    uint8_t dir = 0;
    uint8_t speed_value = 0;

    if (motor == 1 || motor == 2) {
        dir = speed > 0 ? 1 : 2;
        speed_value = map(abs(speed), 0, 255, 0, 100);
    }

    if (motor == 3 || motor == 4) {
        dir = speed > 0 ? 2 : 1;
        speed_value = map(abs(speed), 0, 255, 0, 100);
    }

    if (speed_value == 40) speed_value = 0;

    Serial_2->write(motor);
    Serial_2->write(dir);
    Serial_2->write(speed_value);
    Serial_2->write('\r');
    Serial_2->write('\n');
}

void ACB_SmartCar_V2::Move(int Dir, int Speed)
{
    if (Dir == 163) {
        motorControl(1, Speed);
        motorControl(2, Speed);
        motorControl(3, Speed);
        motorControl(4, Speed);
    }

    if (Dir == 92) {
        motorControl(1, -Speed);
        motorControl(2, -Speed);
        motorControl(3, -Speed);
        motorControl(4, -Speed);
    }

    if (Dir == 149) {
        motorControl(1, Speed);
        motorControl(2, -Speed);
        motorControl(3, -Speed);
        motorControl(4, Speed);
    }

    if (Dir == 106) {
        motorControl(1, -Speed);
        motorControl(2, Speed);
        motorControl(3, Speed);
        motorControl(4, -Speed);
    }

    if (Dir == 172) {
        motorControl(1, Speed);
        motorControl(2, Speed);
        motorControl(3, -Speed);
        motorControl(4, -Speed);
    }

    if (Dir == 83) {
        motorControl(1, -Speed);
        motorControl(2, -Speed);
        motorControl(3, Speed);
        motorControl(4, Speed);
    }

    if (Dir == 34) {
        motorControl(1, 0);
        motorControl(2, Speed);
        motorControl(3, Speed);
        motorControl(4, 0);
    }

    if (Dir == 129) {
        motorControl(1, Speed);
        motorControl(2, 0);
        motorControl(3, 0);
        motorControl(4, Speed);
    }

    if (Dir == 20) {
        motorControl(1, 0);
        motorControl(2, -Speed);
        motorControl(3, -Speed);
        motorControl(4, 0);
    }

    if (Dir == 72) {
        motorControl(1, -Speed);
        motorControl(2, 0);
        motorControl(3, 0);
        motorControl(4, -Speed);
    }

    if (Dir == 0) {
        motorControl(1, 0);
        motorControl(2, 0);
        motorControl(3, 0);
        motorControl(4, 0);
    }
}