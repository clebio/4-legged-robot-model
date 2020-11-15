#include <Wire.h>

#define ENABLE_RELAY 1

const int RELAY_PIN = A0;
bool RELAY_ON = false;

unsigned long loopTime = millis();

const long interval = 300;
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(interval);
    }

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
    delay(interval);
    digitalWrite(RELAY_PIN, LOW);
}

void loop()
{
    if (millis() - loopTime > interval)
    {

        loopTime = millis();
        Serial.println(loopTime);
        RELAY_ON = !RELAY_ON;

        if (ENABLE_RELAY && RELAY_ON)
        {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("Relay on");
        }

        else if (ENABLE_RELAY && !RELAY_ON)
        {
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Relay off");
        }
    }
}
