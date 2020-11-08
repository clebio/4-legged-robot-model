#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ENABLE_RELAY 1
#define ENABLE_IMU 1
#define ENABLE_SERVO 1

#define NUM_SERVOS 12
#define SERVO_FIRST_PIN 53

const int RELAY_PIN = A0;
bool RELAY_ON;

// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime = millis();
const long interval = 200;
const String startMarker = "<";
const String endMarker = ">";

#ifdef ENABLE_SERVO
Servo Servos[NUM_SERVOS];
int pulses[NUM_SERVOS];
#endif
int i = 0;

#ifdef ENABLE_IMU
Adafruit_BNO055 bno;
#endif

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
    if (!ENABLE_RELAY)
    {
        return "RELAY#Disabled";
    }

    if (message == "on")
    {
        Serial.println("Enabling relay");
        RELAY_ON = true;
        digitalWrite(RELAY_PIN, HIGH);
    }
    else if (message == "off")
    {
        RELAY_ON = false;
        digitalWrite(RELAY_PIN, LOW);
    }
    return message;
}

String shutdown()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        Servos[i].detach();
    }
    digitalWrite(RELAY_PIN, LOW);
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

    // if (type == "RELAY")
    // {
        // response = relayctl(message);
    // }
    if (type == "QUIT")
    {
        response = shutdown();
    }
    return response;
}

void sit()
{
    int positions[NUM_SERVOS] = {
        // FL
        90, 110, 120,
        // BL
        90, 160, 110,
        // FR
        90, 60, 80,
        // BR
        90, 45, 70};

    // int angles[] = {80, 100, 90};
    for (i = 0; i < NUM_SERVOS; i++)
    {
        Servos[i].write(positions[i]);
    }
    delay(interval);
}

int SERVO_LIM_LOW[NUM_SERVOS] = {
    // FL
    544, 544, 544,
    // BL
    544, 544, 544,
    // FR
    544, 544, 544,
    // BR
    544, 544, 544};

int SERVO_LIM_HIGH[NUM_SERVOS] = {
    // FL
    2400, 2400, 2400,
    // BL
    2400, 2400, 2400,
    // FR
    2400, 2400, 2400,
    // BR
    2400, 2400, 2400};

void setupServos()
{
    // https://www.arduino.cc/en/Reference/ServoAttach

    // Count up from SERVO_FIRST_PIN by 1
    // for (i = 0; i < NUM_SERVOS; i++)
    // {
    //     Servos[i].attach(SERVO_FIRST_PIN + i, SERVO_LIM_LOW[i], SERVO_LIM_HIGH[i]);
    // }

    // Outer row, short edge
    // for (i = 0; i < NUM_SERVOS; i++)
    // {
    //     // starting at pin 53, count down by two
    //     // to match the outer row digital pins on the short edge of the Mega
    //     Servos[i].attach(31 + 2 * i, SERVO_LIM_LOW[i], SERVO_LIM_HIGH[i]);
    // }

    // Outer and inner row, short edge, six pins on each (half of NUM_SERVOS)
    // In [3]: [53-2*i for i in range(int(NUM_SERVOS/2))]
    // Out[3]: [53, 51, 49, 47, 45, 43]
    // In [8]: [52-2*i for i in range(int(NUM_SERVOS/2))]
    // Out[8]: [52, 50, 48, 46, 44, 42]
    for (i = 0; i < (int)(NUM_SERVOS / 2); i++)
    {
        // starting at pin 53, count down by two
        // (outer row digital pins on the short edge of the Mega)
        Servos[i].attach(53 - 2 * i);
        // Servos[i].attach(53 - 2 * i, SERVO_LIM_LOW[i], SERVO_LIM_HIGH[i]);
    }
    int j = i - (int)(NUM_SERVOS / 2);
    for (i = (int)(NUM_SERVOS / 2); i < NUM_SERVOS; i++)
    {
        j = i - (int)(NUM_SERVOS / 2);
        // starting at pin 52, count down by two
        // (inner row digital pins on the short edge of the Mega)
        Servos[i].attach(52 - 2 * j);
        // Servos[i].attach(52 - 2 * j, SERVO_LIM_LOW[i], SERVO_LIM_HIGH[i]);
    }
    // TODO: do we need to cast NUM_SERVOS / 2 to integer, on Arduino?

    // Jiggle
    // int angles[] = {80, 100, 90};
    // for (a = 0; a < 3; a++) {
    //     for (i = 0; i < NUM_SERVOS; i++)
    //     {
    //         Servos[i].write(angles[a]);
    //     }
    // }
    sit();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(interval);
    }

    if (ENABLE_IMU)
    {
        setup_imu();
    }
    if (ENABLE_RELAY)
    {
        pinMode(RELAY_PIN, OUTPUT);
        // digitalWrite(RELAY_PIN, HIGH);
        relayctl("on");
        // delay(interval);
        // relayctl("off");
    }

    if (ENABLE_SERVO)
    {
        setupServos();
    }
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
        }

        if (millis() - loopTime > interval)
        {
            loopTime = millis();
            if (ENABLE_IMU)
            {
                Serial.println(readIMU());
            }
        }
    }
}
