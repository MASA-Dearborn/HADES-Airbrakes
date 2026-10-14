#include <Arduino.h>
#include <SPI.h>
#include <DRV8434S.h>

/*
Coil Pair A : Red Wire (A1) and Blue Wire (A2)
Coil Pair B : Green Wire (B1) and Black Wire (B2)

SPI Wiring (Teensy 4.0 -> DRV8434S):
13 (SCK) -> SCLK, 11 (MOSI) -> SDI, 12 (MISO) -> SDO, 10 -> nSCS
VSDO and nSLEEP must be tied to 3.3V or the driver will not answer on SPI.
*/

/*
HOW TO RUN:
1.) pio run -e teensy40-stepper -t upload
^ if ur running on a teensy 4.0 and trying to test motor stepping.
2.) pio device monitor -p COM10 -b 115200
3.) Type a command (r, s, d run instantly, others run on Enter or 1 s after you stop typing):
    r    = start the driver
    s    = status (is the driver ok, any faults)
    f200 = move forward 200 steps
    b200 = move backward 200 steps
    d    = raw register dump in hex (only for deep debugging)
*/

/*
TODO:
- Smoother stepping (smooth interpolation)

- Setup Sensors (Make new cpp file like sensors_motor.cpp)
- AS motor is running / stepping, we need to read the sensors and log the data to serial so we can see how the motor is performing.
- Pressure, gyroscrope, IMU, whatever we need on the rocket
*/



// Teensy 4.0 pins
constexpr uint8_t STEP_PIN = 2;
constexpr uint8_t DIR_PIN  = 3;
constexpr uint8_t SCS_PIN  = 10; // Expects an 8 bit integer passed in (Driver Library)

// Motor settings
constexpr int MOTOR_CURRENT_MA = 250;
constexpr uint32_t STEP_DELAY_US = 2000;
constexpr uint32_t MAX_STEPS = 2000; // pls dont blow up motor
constexpr uint32_t delay_start_ms = 10; // Delay before initializing the driver after motor supply is powered. (ms)

// Driver init / health settings
constexpr uint32_t DRV_SPI_HZ = 500000;       // Must match the SPI speed hardcoded in the DRV8434S library
constexpr uint8_t INIT_ATTEMPTS = 5;          // r quietly retries this many times before giving up
constexpr uint32_t INIT_RETRY_DELAY_MS = 50;  // Wait between attempts so motor power can settle
constexpr uint32_t ENABLE_SETTLE_MS = 5;      // Let the charge pump come up before checking faults
constexpr uint32_t HEALTH_CHECK_MS = 250;     // How often (while idle) we check the driver still has our settings
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000; // Run a typed command this long after the last key if Enter never comes

// One short line per DRV8434S fault bit.
struct FaultInfo { DRV8434SFaultBit bit; const char *message; };
constexpr FaultInfo FAULTS[] = {
    {DRV8434SFaultBit::UVLO,      "FAULT: Motor power is off or too low."},
    {DRV8434SFaultBit::CPUV,      "FAULT: Motor power is too weak for the driver."},
    {DRV8434SFaultBit::OCP,       "FAULT: Overcurrent. Check for shorted motor wires."},
    {DRV8434SFaultBit::TF,        "FAULT: Driver is overheating."},
    {DRV8434SFaultBit::SPI_ERROR, "FAULT: Bad SPI data. Check SPI wires and ground."},
    {DRV8434SFaultBit::OL,        "FAULT: Motor wire disconnected."},
    {DRV8434SFaultBit::STL,       "FAULT: Motor stalled."},
};
// Faults where clearing and retrying over and over could damage the driver.
constexpr uint8_t DANGEROUS_FAULTS = (1 << (uint8_t)DRV8434SFaultBit::OCP) | (1 << (uint8_t)DRV8434SFaultBit::TF);

// Printed at startup and by s, so you can tell if the board is actually running the code you just uploaded.
constexpr const char *BUILD_STAMP = "Firmware: " __DATE__ " " __TIME__;
constexpr const char *NO_RESPONSE = "Driver not responding. Check motor power and SPI wires.";

DRV8434S driver;
bool driverReady = false;
const char *lastInitError = nullptr; // Why the last start failed (nullptr = never run or it worked)

uint32_t stepsRemaining = 0;
uint32_t lastStepTime = 0;
bool stepPinState = false;
uint32_t lastHealthCheckMs = 0;

// Move the motor by a specified number of steps
inline void moveMotor(uint32_t steps, bool direction)
{
        /*
        Step the motor Logic
       - Set The Voltage on the step pin to High.
       - the motor only steps based on the pulse in voltage, not current state.
       - this is why we have to set it to low after a delay, so the motor can step again.
       - The delay is important, if the delay is too short, the motor will not step
       - so for each iteration in the step, its simple, set voltage high then wait to let it react.
       - Then reset it for the next iteration so it can react to the increase in voltage.
        */

        digitalWrite(DIR_PIN, direction);
        stepsRemaining = steps * 2;
        lastStepTime = micros();
        stepPinState = false;
        digitalWrite(STEP_PIN,LOW);
}

void updateMotor(){
    if (stepsRemaining == 0) return;
    uint32_t currentTime = micros();

    // Only step the motor if the delay has passed since the last step
    if ((currentTime - lastStepTime) >= STEP_DELAY_US){
        stepPinState = !stepPinState; // flip step pin state (low -> high or high -> low)
        digitalWrite(STEP_PIN,stepPinState);
        lastStepTime = currentTime; // Only update last step when we do some action / logic.
        stepsRemaining--;

        if (stepsRemaining == 0){
            Serial.println("Movement Complete.");
        }
    }
}

// Build the command one character at a time so the motor keeps stepping while you type.
// (readStringUntil used to block for up to 1 s, which froze the motor mid-move.)
// A command is done when you press Enter, when you type a single-key command (no Enter needed),
// or when you stop typing for COMMAND_TIMEOUT_MS (for monitors that never send Enter).
bool readCommand(String &command)
{
    static String buffer;
    static uint32_t lastCharMs = 0;
    bool done = false;

    while (!done && Serial.available())
    {
        char c = Serial.read();
        lastCharMs = millis();
        if (c == '\n' || c == '\r')
        {
            done = buffer.length() > 0; // Enter (blank lines and the \n of a \r\n pair are skipped)
        }
        else
        {
            buffer += c;
            done = buffer == "r" || buffer == "s" || buffer == "d"; // Single-key commands run right away
        }
    }

    // Some monitors never send Enter, so finish the command once you stop typing.
    if (!done && buffer.length() > 0 && millis() - lastCharMs >= COMMAND_TIMEOUT_MS) done = true;
    if (!done) return false;

    command = buffer;
    buffer = "";
    command.trim();
    return true;
}

// The DRV8434S always sends 1s in the top two bits of its status byte, so anything else means nobody answered.
bool driverResponding()
{
    uint8_t ctrl2 = driver.driver.readReg(DRV8434SRegAddr::CTRL2);
    return (driver.driver.lastStatus & 0xC0) == 0xC0 && ctrl2 != 0xFF;
}

// Raw hex dump of every driver register, only for deep debugging (d command).
void printRegisters()
{
    Serial.printf("Raw: status=%02X FAULT=%02X DIAG1=%02X DIAG2=%02X CTRL1-7=",
                  driver.driver.lastStatus, driver.readFault(), driver.readDiag1(), driver.readDiag2());
    for (uint8_t reg = (uint8_t)DRV8434SRegAddr::CTRL1; reg <= (uint8_t)DRV8434SRegAddr::CTRL7; reg++)
    {
        Serial.printf("%02X ", driver.driver.readReg(reg));
    }
    Serial.println();
}

// Read back every control register and check it matches what we wrote.
bool settingsMatch()
{
    for (uint8_t reg = (uint8_t)DRV8434SRegAddr::CTRL1; reg <= (uint8_t)DRV8434SRegAddr::CTRL7; reg++)
    {
        if (driver.driver.readReg(reg) != driver.getCachedReg((DRV8434SRegAddr)reg)) return false;
    }
    return true;
}

// Fault bits we know how to explain (bit 7 only says "some fault is active").
uint8_t activeFaults()
{
    uint8_t faults = driver.readFault();
    uint8_t active = 0;
    for (const FaultInfo &f : FAULTS)
    {
        active |= faults & (1 << (uint8_t)f.bit);
    }
    return active;
}

void printFaults(uint8_t faults)
{
    for (const FaultInfo &f : FAULTS)
    {
        if (faults & (1 << (uint8_t)f.bit)) Serial.println(f.message);
    }
}

// Write all settings, enable the outputs, and confirm the driver actually took them.
// Retries quietly (this is what spamming r used to do by hand) and only prints faults once at the end.
// Returns which attempt worked, or 0 if they all failed (reason in lastInitError).
uint8_t configureDriver()
{
    uint8_t faults = 0;
    for (uint8_t attempt = 1; attempt <= INIT_ATTEMPTS; attempt++)
    {
        if (attempt > 1) delay(INIT_RETRY_DELAY_MS);
        faults = 0;

        driver.resetSettings(); // Also turns the outputs off (EN_OUT = 0)
        driver.setCurrentMilliamps(MOTOR_CURRENT_MA);
        driver.setStepMode(1);
        driver.disableSPIStep();
        driver.disableSPIDirection();
        driver.clearFaults();

        if (!driverResponding())
        {
            lastInitError = NO_RESPONSE;
            continue;
        }
        if (!settingsMatch())
        {
            lastInitError = "Driver settings did not stick. Check SPI wires and ground.";
            continue;
        }

        driver.enableDriver(); // SPI good? enable.
        delay(ENABLE_SETTLE_MS);
        faults = activeFaults();

        if (!settingsMatch())
        {
            lastInitError = "Driver reset when turned on. Motor power may be too weak.";
            continue;
        }
        if (faults == 0)
        {
            lastInitError = nullptr;
            return attempt;
        }

        driver.disableDriver();
        lastInitError = "Driver has a fault.";
        if (faults & DANGEROUS_FAULTS) break; // Don't keep clearing a fault that can damage the driver
    }

    driver.disableDriver();
    printFaults(faults);
    return 0;
}

// A motor supply dip (like changing the voltage) resets the DRV8434S to defaults with its outputs off.
// Catch that and put our settings back instead of making you re-run r.
bool ensureDriverHealthy()
{
    if (!driverReady) return false;
    if (settingsMatch()) return true;

    Serial.println("WARN: Driver lost power. Restoring...");
    driverReady = configureDriver() > 0;
    if (driverReady) Serial.println("OK: Driver restored.");
    else Serial.printf("FAIL: %s\n", lastInitError);
    return driverReady;
}

// Works before r too, so you can check wiring and power first.
void printStatus()
{
    Serial.println(BUILD_STAMP);
    if (driverReady && !ensureDriverHealthy()) return; // It already printed what went wrong

    if (!driverResponding())
    {
        Serial.printf("FAIL: %s\n", NO_RESPONSE);
        return; // Fault bits are garbage if the driver is not answering
    }

    if (driverReady) Serial.println("OK: Driver ready.");
    else if (lastInitError != nullptr) Serial.printf("FAIL: Last start failed. %s\n", lastInitError);
    else Serial.println("Driver is responding. Press r to start it.");

    uint8_t faults = activeFaults();
    if (faults == 0) Serial.println("Faults: none");
    else printFaults(faults);
}


void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {} // Wait (up to 3 s) for the serial monitor so the startup messages are not lost

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    // Drive CS high before SPI starts so the driver never sees half a message while the pins switch over.
    driver.setChipSelectPin(SCS_PIN);
    SPI.begin();

    // The library pulls CS low *before* applying its SPI settings, so put SPI in the driver's
    // mode/speed now or the very first message goes out while the Teensy is still reconfiguring SPI.
    SPI.beginTransaction(SPISettings(DRV_SPI_HZ, MSBFIRST, SPI_MODE1));
    SPI.endTransaction();

    Serial.println("Stepper test ready.");
    Serial.println(BUILD_STAMP);
    Serial.println("Commands: r = start driver, s = status, f200 / b200 = move");
}

void loop()
{
    updateMotor();

    // Only check while idle, the SPI reads would add jitter to the step timing mid-move.
    if (stepsRemaining == 0 && millis() - lastHealthCheckMs >= HEALTH_CHECK_MS)
    {
        lastHealthCheckMs = millis();
        ensureDriverHealthy();
    }

    String command;
    if (!readCommand(command))
        return;

    Serial.print("> "); Serial.println(command); // Echo so you can see what the Teensy actually received

    if (command == "r")
    {
        stepsRemaining = 0; // Stop any move before touching the driver
        digitalWrite(STEP_PIN, LOW);

        // Configure after motor supply is powered
        delay(delay_start_ms);
        Serial.println("Starting driver...");
        uint8_t tries = configureDriver();
        driverReady = tries > 0;

        if (tries == 1) Serial.println("OK: Driver ready.");
        else if (tries > 1) Serial.printf("OK: Driver ready (took %u tries).\n", tries);
        else Serial.printf("FAIL: %s\n", lastInitError);
        return;
    }

    if (command == "s")
    {
        printStatus();
        return;
    }

    if (command == "d")
    {
        printRegisters();
        return;
    }

    char direction = command.charAt(0);
    if (direction != 'f' && direction != 'b')
    {
        Serial.println("Unknown command. Use r, s, f200, or b200.");
        return;
    }

    if (!driverReady)
    {
        Serial.println("Press r first to start the driver.");
        return;
    }
    if (!ensureDriverHealthy()) return;

    long steps = command.substring(1).toInt();

    if (steps <= 0 || steps > (long)MAX_STEPS)
    {
        Serial.printf("Steps must be 1 to %lu.\n", (unsigned long)MAX_STEPS);
        return;
    }

    Serial.printf("Moving %s %ld steps...\n", direction == 'f' ? "forward" : "backward", steps);
    moveMotor(steps, direction == 'f' ? HIGH : LOW);
}
