#include "MPU9250.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup();
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();
        mpu.print();

        Serial.print("roll : ");
        Serial.println(mpu.getRoll());
        Serial.print("pitch : ");
        Serial.println(mpu.getPitch());
        Serial.print("yaw : ");
        Serial.println(mpu.getYaw());

        prev_ms = millis();
    }
}

