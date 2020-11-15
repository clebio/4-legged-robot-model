#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime = 0;
const long interval = 500;
const String startMarker = "<";
const String endMarker = ">";
Adafruit_BNO055 bno;

String readIMU()
{
    /* Get BNO-055 IMU data
    
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

    sensors_event_t event;
    float ex, ey, ez, ax, ay, az, qx, qy, qz, qw;

    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    ex = event.orientation.x;
    ey = event.orientation.y;
    ez = event.orientation.z;

    bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;

    imu::Quaternion quat = bno.getQuat();
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    qw = quat.w();

    /* Also send calibration data for each sensor. */
    // uint8_t sys, gyro, accel, mag = 0;
    // response.concat(sys, DEC);
    // response.concat(gyro, DEC);
    // response.concat(accel, DEC);
    // response.concat(mag, DEC);

    // "BNO055~%s %s %s %s %s %s %s",
    // where each %s is dtostrf(ex, 4,2) = 6 char plus the decimal place, plus the "#" character separator == 8 char
    // "BNO055~".length() == 7, ffmt * six vars = 48
    // resp_len = 7+ 48 = 55 (but maybe we should still pad this?)
    // int resp_len = sizeof(ex) + sizeof(ey) + ...;
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

    char fqx[width + precision + 1];
    dtostrf(qx, width, precision, fqx);

    char fqy[width + precision + 1];
    dtostrf(qy, width, precision, fqy);

    char fqz[width + precision + 1];
    dtostrf(qz, width, precision, fqz);

    char fqw[width + precision + 1];
    dtostrf(qw, width, precision, fqw);

    sprintf(response, "BNO055#%s %s %s %s %s %s", fex, fey, fez, fax, fay, faz);
    // sprintf(response, "BNO055#%s %s %s %s %s %s %s", fex, fey, fez, fqx, fqy, fqz, fqw);
    // sprintf(response, "BNO055#%s %s %s %s %s %s %s", fax, fay, faz, fqx, fqy, fqz, fqw);
    return response;
}

void setup()
{
    bno = Adafruit_BNO055(55, 0x28);
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    bno.setExtCrystalUse(true);

    Serial.begin(115200);
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
