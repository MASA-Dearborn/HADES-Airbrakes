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
3.) Type a command (r and s run instantly, others run on Enter or 1 s after you stop typing):
    r    = initialize the driver
    s    = print driver status and faults
    f200 = move forward 200 steps
    b200 = move backward 200 steps
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
constexpr uint8_t INIT_ATTEMPTS = 5;          // r retries this many times before giving up
constexpr uint32_t INIT_RETRY_DELAY_MS = 50;  // Wait between attempts so motor power can settle
constexpr uint32_t ENABLE_SETTLE_MS = 5;      // Let the charge pump come up before checking faults
constexpr uint32_t HEALTH_CHECK_MS = 250;     // How often (while idle) we check the driver still has our settings
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000; // Run a typed command this long after the last key if Enter never comes

// One plain-English sentence per DRV8434S fault bit (abbreviation in brackets matches the datasheet).
struct FaultInfo { DRV8434SFaultBit bit; const char *message; };
constexpr FaultInfo FAULTS[] = {
    {DRV8434SFaultBit::UVLO,      "Fault: motor supply (VM) is off or dropped too low (UVLO)."},
    {DRV8434SFaultBit::CPUV,      "Fault: driver charge pump is too low, usually from a weak motor supply (CPUV)."},
    {DRV8434SFaultBit::OCP,       "Fault: overcurrent, a motor wire may be shorted or touching another wire (OCP)."},
    {DRV8434SFaultBit::TF,        "Fault: driver is overheating, let it cool or lower the current (TF)."},
    {DRV8434SFaultBit::SPI_ERROR, "Fault: driver got a garbled SPI message, check SPI wiring and ground (SPI_ERROR)."},
    {DRV8434SFaultBit::OL,        "Fault: open load, a motor coil wire is disconnected (OL)."},
    {DRV8434SFaultBit::STL,       "Fault: motor stall detected (STL)."},
};
// Faults where clearing and retrying over and over could damage the driver.
constexpr uint8_t DANGEROUS_FAULTS = (1 << (uint8_t)DRV8434SFaultBit::OCP) | (1 << (uint8_t)DRV8434SFaultBit::TF);

// Printed at startup and by s, so you can tell if the board is actually running the code you just uploaded.
constexpr const char *BUILD_STAMP = "Stepper test firmware built " __DATE__ " " __TIME__ ".";
constexpr const char *NOT_ANSWERING = "driver is not answering on SPI, check motor power (VM), VSDO/nSLEEP, and the SPI wires.";

DRV8434S driver;
bool driverReady = false;
const char *lastInitError = nullptr; // Why the last r failed (nullptr = never run or it worked)

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
// A command is done when you press Enter, when you type r or s (no Enter needed),
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
            done = buffer == "r" || buffer == "s"; // Single-key commands run right away
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

// Dump every driver register on one line so the raw values can be pasted when asking for help.
void printRegisters()
{
    Serial.printf("Registers: status=0x%02X FAULT=0x%02X DIAG1=0x%02X DIAG2=0x%02X CTRL1-7=",
                  driver.driver.lastStatus, driver.readFault(), driver.readDiag1(), driver.readDiag2());
    for (uint8_t reg = (uint8_t)DRV8434SRegAddr::CTRL1; reg <= (uint8_t)DRV8434SRegAddr::CTRL7; reg++)
    {
        Serial.printf("%02X ", driver.driver.readReg(reg));
    }
    Serial.println();
}

// Read back every control register and say which one does not match what we wrote.
bool settingsMatch()
{
    for (uint8_t reg = (uint8_t)DRV8434SRegAddr::CTRL1; reg <= (uint8_t)DRV8434SRegAddr::CTRL7; reg++)
    {
        uint8_t expected = driver.getCachedReg((DRV8434SRegAddr)reg);
        uint8_t actual = driver.driver.readReg(reg);
        if (actual != expected)
        {
            Serial.printf("Register CTRL%u reads 0x%02X but should be 0x%02X.\n", reg - 2, actual, expected);
            return false;
        }
    }
    return true;
}

// Print each active fault as one sentence and return the fault bits that were set.
uint8_t printFaults()
{
    uint8_t faults = driver.readFault();
    uint8_t active = 0;
    for (const FaultInfo &f : FAULTS)
    {
        uint8_t mask = 1 << (uint8_t)f.bit;
        if (faults & mask)
        {
            Serial.println(f.message);
            active |= mask;
        }
    }
    return active;
}

// Remember why an init attempt failed so r and s can both say it in one line.
void initFailed(uint8_t attempt, const char *why)
{
    lastInitError = why;
    Serial.printf("Attempt %u of %u failed: %s\n", attempt, INIT_ATTEMPTS, why);
}

// Write all settings, enable the outputs, and confirm the driver actually took them.
// Retries on its own, this is what spamming r used to do by hand.
bool configureDriver()
{
    for (uint8_t attempt = 1; attempt <= INIT_ATTEMPTS; attempt++)
    {
        if (attempt > 1) delay(INIT_RETRY_DELAY_MS);

        driver.resetSettings(); // Also turns the outputs off (EN_OUT = 0)
        driver.setCurrentMilliamps(MOTOR_CURRENT_MA);
        driver.setStepMode(1);
        driver.disableSPIStep();
        driver.disableSPIDirection();
        driver.clearFaults();

        if (!driverResponding())
        {
            initFailed(attempt, NOT_ANSWERING);
            continue;
        }
        if (!settingsMatch())
        {
            initFailed(attempt, "driver did not keep the settings we wrote, SPI data is probably getting corrupted.");
            continue;
        }

        driver.enableDriver(); // SPI good? enable.
        delay(ENABLE_SETTLE_MS);

        if (!settingsMatch())
        {
            printFaults();
            initFailed(attempt, "driver reset right after enabling, the motor supply is probably sagging under load.");
            continue;
        }

        uint8_t faults = printFaults();
        if (faults == 0)
        {
            if (attempt > 1) Serial.printf("Driver came up on attempt %u.\n", attempt);
            lastInitError = nullptr;
            return true;
        }

        driver.disableDriver();
        if (faults & DANGEROUS_FAULTS)
        {
            initFailed(attempt, "driver reported a fault that can damage it (see the Fault line above), so we stopped retrying.");
            return false;
        }
        initFailed(attempt, "driver reported a fault after enabling (see the Fault line above).");
    }

    driver.disableDriver();
    printRegisters();
    return false;
}

// A motor supply dip (like changing the voltage) resets the DRV8434S to defaults with its outputs off.
// Catch that and put our settings back instead of making you re-run r.
bool ensureDriverHealthy()
{
    if (!driverReady) return false;
    if (settingsMatch()) return true;

    Serial.println("Driver lost its settings (motor power probably dipped or changed), restoring them now.");
    printFaults();
    driverReady = configureDriver();
    if (driverReady) Serial.println("Driver restored. Ready to move!");
    else Serial.printf("Could not restore the driver: %s Fix that then enter r.\n", lastInitError);
    return driverReady;
}

// Works before r too, so you can check wiring and power first.
void printStatus()
{
    Serial.println(BUILD_STAMP);

    if (!driverResponding()) Serial.printf("SPI: %s\n", NOT_ANSWERING);
    else Serial.println("SPI: driver is answering.");

    if (driverReady)
    {
        if (settingsMatch()) Serial.println("Init: driver is initialized and its settings look correct.");
    }
    else if (lastInitError != nullptr) Serial.printf("Init: not initialized, last r failed because %s\n", lastInitError);
    else Serial.println("Init: r has not been run yet.");

    if (printFaults() == 0) Serial.println("Faults: none reported.");
    printRegisters();
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

    Serial.println(BUILD_STAMP);
    Serial.println("Stepper test ready.");
    Serial.println("Enter r to initialize the driver, or s for driver status.");
    Serial.println("Then enter f200 or b200 to move.");
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
        Serial.println("Initializing driver...");
        driverReady = configureDriver();
        if (driverReady) Serial.println("Driver initialized. Ready to move!");
        else Serial.printf("Driver init failed: %s\n", lastInitError);
        return;
    }

    if (command == "s")
    {
        printStatus();
        return;
    }

    char direction = command.charAt(0);
    if (direction != 'f' && direction != 'b')
    {
        Serial.println("Unknown command, use r, s, f<steps>, or b<steps> (example: f200).");
        return;
    }

    if (!driverReady)
    {
        Serial.println("Initialize with r first.");
        return;
    }
    if (!ensureDriverHealthy()) return;

    long steps = command.substring(1).toInt();

    if (steps <= 0 || steps > (long)MAX_STEPS)
    {
        Serial.printf("Choose 1 to %lu steps (example: f200).\n", (unsigned long)MAX_STEPS);
        return;
    }

    if (stepsRemaining > 0) Serial.println("New move received, replacing the current move.");
    Serial.printf("Moving %s %ld steps...\n", direction == 'f' ? "forward" : "backward", steps);
    moveMotor(steps, direction == 'f' ? HIGH : LOW);
}
