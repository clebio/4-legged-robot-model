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

void parseControls(String data)
{
    int endIdx = data.indexOf(endMarker);
    int spc = 1;
    while (spc <= endIdx || spc != -1)
    {
        int next = data.indexOf(spaceMarker, spc);
        if (next == -1)
        {
            break;
        }
        String word = data.substring(spc, next);
        spc = next + 1;
        Serial.print(word + " ");
    }
    Serial.println("");
}

void loop()
{
    if (Serial.available() > 0)
    {
        String data = Serial.readStringUntil('\n');

        if (data.startsWith(startMarker) && data.indexOf(endMarker) > 0)
        {
            Serial.print("Found control markers: ");
            parseControls(data);
        }
        else
        {
            Serial.print("You sent me: ");
            Serial.println(data);
        }
    }
}
