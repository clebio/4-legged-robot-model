// https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/#Bidirectional_Serial_communication_between_Raspberry_Pi_and_Arduino
unsigned long loopTime;

unsigned long previousMillis = 0;
const long interval = 20;
const String startMarker = "<";
const String endMarker = ">";
const String spaceMarker = "#";

void setup()
{
    Serial.begin(115200);
}

int loopIdx = 0;
int maxLoops = 50;
void loop()
{
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');

        int endIdx = data.indexOf(endMarker);
        if (data.startsWith(startMarker) && endIdx > 0)
        {
            Serial.print("Found control markers: ");
            Serial.print(data);
            Serial.println("");
        }
        else
        {
            Serial.print("You sent me: ");
            Serial.println(data);
        }
    }
}
