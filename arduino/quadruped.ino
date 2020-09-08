#include <Servo.h>

// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime;
unsigned long previousMillis = 0;
const long interval = 20;
const String startMarker = "<";
const String endMarker = ">";

#define NUM_SERVOS 12
Servo Servos[NUM_SERVOS];
int pulses[NUM_SERVOS];
int i = 0;

void setup()
{
    Serial.begin(115200);
    {
        for (i = 0; i < NUM_SERVOS; i++)
        {
            Servos[i].attach(2 + i);
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
    String wordSep = "#";
    String movSep = "~";
    String word = "";
    int wordIdx = 0;
    int spc = 1;
    String response = "FAIL";

    // Serial.println(instructions);
    // #0~1239#1~1893#2~397#3~1570#4~439#5~1887#6~1484#7~379#8~658#9~871#10~1202#11~786
    if (NUM_SERVOS == 0)
    {
        return "No servos configured";
    }
    response = "";

    while (wordIdx < NUM_SERVOS && spc > 0)
    {
        int next = instructions.indexOf(wordSep, spc);
        word = instructions.substring(spc, next);

        int movIdx = word.indexOf(movSep);
        int servo = word.substring(0, movIdx).toInt();
        String distance = word.substring(movIdx + 1);

        response.concat(" * ");
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
            String response = "<";
            response.concat(dispatch(control));
            response.concat(">");
            Serial.println(response);
        }
        else
        {
            // Data received is not control, so just echo whatever it is
            // Serial.print("You sent: ");
            // Serial.println(data);
        }
    }

    // When we have IMU data, send that as a "<DATA...>" record
}
