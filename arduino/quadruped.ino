#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ENABLE_RELAY 1
#define ENABLE_IMU 0
#define ENABLE_SERVO 0
#define NUM_SERVOS 12
const int RELAY_PIN = A0;

// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime = millis();
unsigned long previousMillis = 0;
const long interval = 200;
const String startMarker = "<";
const String endMarker = ">";

Servo Servos[NUM_SERVOS];
int pulses[NUM_SERVOS];
int i = 0;

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

    sprintf(response, "BNO055#%s %s %s %s %s %s", fex, fey, fez, fax, fay, faz); // fqx, fqy, fqz, fqw);
    return response;
}

void setup_imu()
{
    bno = Adafruit_BNO055(55, 0x28);
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    bno.setExtCrystalUse(true);
}

String relayctl(String message)
{
    bool _RELAYED = LOW;
    if (!ENABLE_RELAY)
    {
        return "RELAY#Disabled";
    }
    if (message == "on" || message == "#on")
    {
        _RELAYED = HIGH;
    }
    digitalWrite(RELAY_PIN, _RELAYED);
    return String(_RELAYED);
}

void setup()
{
    if (ENABLE_IMU)
    {
        setup_imu();
    }
    if (ENABLE_RELAY)
    {
        pinMode(RELAY_PIN, OUTPUT);
    }

    Serial.begin(115200);
    while (!Serial)
    {
        delay(interval);
    }
    {
        if (ENABLE_SERVO)
        {
            for (i = 0; i < NUM_SERVOS; i++)
            {
                Servos[i].attach(2 + i);
            }
        }
    }
}

String shutdown()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        Servos[i].detach();
    }
    return "Shutdown complete";
}

String moveServos(String instructions)
{
    if (!ENABLE_SERVO)
    {
        return "SERVOS#Disabled";
    }
    /*
        What println(instructions) looks like:
        #0~1239#1~1893#2~397#3~1570#4~439#5~1887#6~1484#7~379#8~658#9~871#10~1202#11~786
    */
    String wordSep = "#";
    String movSep = "~";
    String word = "";
    int wordIdx = 0;
    int spc = 1;
    String response = "FAIL";

    if (NUM_SERVOS == 0)
    {
        return "No servos configured";
    }
    response = "SERVOS#";

    while (wordIdx < NUM_SERVOS && spc > 0)
    {
        int next = instructions.indexOf(wordSep, spc);
        word = instructions.substring(spc, next);

        int movIdx = word.indexOf(movSep);
        int servo = word.substring(0, movIdx).toInt();
        String distance = word.substring(movIdx + 1);

        response.concat(" ");
        response.concat(servo);
        response.concat("->");
        response.concat(distance);

        spc = next + 1;
        wordIdx++;
        Servos[servo].write(distance.toInt());
    }
    return response;
}

String dispatch(String data)
{
    // Separate data into the first part and remainder.
    //
    // First part is the type, second part contains arguments for that type of command
    // Examples:
    //   <SERVO#1~15000#2~700>
    //   <QUIT>
    //   <BLINK_LEDS#1,4,5>
    //
    String spaceMarker = "#";
    int nextIdx = data.indexOf(spaceMarker);
    if (nextIdx < 0)
    {
        // There is no space separator, meaning we expect data to be a single command, like "<SHUTDOWN>"
        nextIdx = data.length();
    }

    String type = data.substring(0, nextIdx);
    String message = data.substring(nextIdx, data.length());

    String response = "";
    if (type == "SERVO")
    {
        response = moveServos(message);
    }
    if (type == "RELAY")
    {
        response = relayctl(message);
    }
    if (type == "QUIT")
    {
        response = shutdown();
    }
    return response;
}

void loop()
{
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');
        if (data.startsWith(startMarker) && data.indexOf(endMarker) > 0)
        {

            // The data we read is a control string, "<...>"
            String control = data.substring(1, data.length() - 1);
            String response = dispatch(control);
            if (response != "")
            {
                Serial.println(response);
            }
            if (ENABLE_IMU)
            {
                Serial.println(readIMU());
            }
        }
    }
}
