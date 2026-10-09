#include <Arduino.h>
#include <SPI.h>
#include <DRV8434S.h>

/*
Coil Pair A : Red Wire (A1) and Blue Wire (A2)
Coil Pair B : Green Wire (B1) and Black Wire (B2)
*/

/*
HOW TO RUN:
1.) pio run -e teensy40-stepper -t upload
^ if ur running on a teensy 4.0 and trying to test motor stepping.
2.) pio device monitor -p COM10 -b 115200
*/

/*
TODO:
- More Logging, We need debugging to be a bit more obvious as to why xyz is failing or not doing something we want.
- Upon First Trying (Clicking R to run) Sometimes it will FAIL then work if u spam r? that needs fixing asap it should always work on first try lol imagine in comp it fucking fails [crying emoji]
- if you change the voltage, you have to reinitialize the driver / program so u can see the changes, needs a fix.
- Smoother stepping (smooth interpolation)
*/

// Teensy 4.0 pins
constexpr int STEP_PIN = 2;
constexpr int DIR_PIN  = 3;
constexpr int SCS_PIN  = 10;

// Motor settings
constexpr int MOTOR_CURRENT_MA = 250;
constexpr int STEP_DELAY_US = 2000;
constexpr int MAX_STEPS = 2000; // pls dont blow up motor



DRV8434S driver;
bool driverReady = false;

// Move the motor by a specified number of steps
void moveMotor(int steps, bool direction)
{
    digitalWrite(DIR_PIN, direction);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY_US);

        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY_US);
    }
}


void setup()
{
    Serial.begin(115200);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    SPI.begin();
    driver.setChipSelectPin(SCS_PIN);

    Serial.println("Stepper test ready.");
    Serial.println("Enter r to initialize the driver.");
    Serial.println("Then enter f200 or b200 to move.");
}

void loop()
{
    if (!Serial.available())
        return;

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "r")
    {
        // Configure after motor supply is powered
        driverReady = false;
        driver.resetSettings();
        driver.disableDriver();
        driver.setCurrentMilliamps(MOTOR_CURRENT_MA);
        driver.setStepMode(1);
        driver.disableSPIStep();
        driver.disableSPIDirection();
        driver.clearFaults();

        if (!driver.verifySettings())
        {
            Serial.println("SPI verification failed. Check wiring and power.");
            return;
        }

        driver.enableDriver();

        if (!driver.verifySettings())
        {
            driver.disableDriver();
            Serial.println("Driver enable verification failed.");
            return;
        }

        driverReady = true;
        Serial.println("Driver initialized. Ready to move!");
        return;
    }

    if (!driverReady)
    {
        Serial.println("Initialize with r first.");
        return;
    }

    int steps = command.substring(1).toInt();

    if (steps <= 0 || steps > MAX_STEPS)
    {
        Serial.println("Choose 1 to 2000 steps.");
        return;
    }

    if (command.startsWith("f"))
    {
        Serial.println("Forward");
        moveMotor(steps, HIGH);
    }
    else if (command.startsWith("b"))
    {
        Serial.println("Backward");
        moveMotor(steps, LOW);
    }
}
