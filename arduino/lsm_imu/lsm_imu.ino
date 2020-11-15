#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime = 0;
const long interval = 500;
const String startMarker = "<";
const String endMarker = ">";

Adafruit_LSM9DS1 lsm;
String readIMU()
{
    /* Get LSM9DS1 IMU data
    
    Based off of https://github.com/adafruit/Adafruit_BNO055/blob/master/examples/webserial_3d/webserial_3d.ino
    Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
    */

    float ex, ey, ez, ax, ay, az, qx, qy, qz, qw;
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    ex = g.gyro.x;
    ey = g.gyro.y;
    ez = g.gyro.z;

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    int resp_len = 80;
    char response[resp_len];
    int width = 4;
    int precision = 2;

    // TODO: Couldn't figure out how to make a method for this
    char fex[width + precision + 1];
    dtostrf(ex, width, precision, fex);

    char fey[width + precision + 1];
    dtostrf(ey, width, precision, fey);

    char fez[width + precision + 1];
    dtostrf(ez, width, precision, fez);

    char fax[width + precision + 1];
    dtostrf(ax, width, precision, fax);

    char fay[width + precision + 1];
    dtostrf(ay, width, precision, fay);

    char faz[width + precision + 1];
    dtostrf(az, width, precision, faz);

    sprintf(response, "LSM9DS1#%s %s %s %s %s %s", fex, fey, fez, fax, fay, faz);
    // sprintf(response, "LSM9DS1#%s %s %s %s %s %s %s", fex, fey, fez, fqx, fqy, fqz, fqw);
    // sprintf(response, "LSM9DS1#%s %s %s %s %s %s %s", fax, fay, faz, fqx, fqy, fqz, fqw);
    return response;
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        delay(1); // will pause Zero, Leonardo, etc until serial console opens
    }

    Serial.println("LSM9DS1 data read demo");

    if (!lsm.begin())
    {
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        while (1)
            ;
    }
    Serial.println("Found LSM9DS1 9DOF");

    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

String response = "FAIL";
void loop()
{
    if (millis() - loopTime >= interval)
    {
        loopTime = millis();
        Serial.print("IMU:");
        response = readIMU();
        Serial.println(response);
    }
}
