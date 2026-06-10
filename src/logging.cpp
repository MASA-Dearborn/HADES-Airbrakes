#include "logging.h"
#include "config.h"
#include <Arduino.h>
#include <SdFat.h>

static SdFs   sd;
static FsFile file;
static bool   s_ready      = false;
static uint32_t s_writeCount = 0;

bool loggerBegin() {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SD: init failed — logging disabled");
        return false;
    }

    // Find the next unused filename FLT00.BIN … FLT99.BIN
    char fname[13];
    int  slot = -1;
    for (int i = 0; i < 100; i++) {
        snprintf(fname, sizeof(fname), "FLT%02d.BIN", i);
        if (!sd.exists(fname)) { slot = i; break; }
    }
    if (slot < 0) {
        Serial.println("SD: no free log slots (FLT00–FLT99 all exist)");
        return false;
    }

    if (!file.open(fname, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.println("SD: file open failed");
        return false;
    }

    // Pre-allocate contiguous space to eliminate cluster-allocation delays
    // during flight.  Non-fatal if the card doesn't support it.
    if (!file.preAllocate(LOG_PREALLOC_BYTES)) {
        Serial.println("SD: preAllocate skipped");
    }

    // Rewind to the start of the pre-allocated region so writes begin at byte 0
    file.rewind();

    Serial.print("SD: logging to ");
    Serial.println(fname);
    Serial.print("SD: record size = ");
    Serial.print((int)sizeof(LogRecord));
    Serial.println(" bytes");

    s_ready      = true;
    s_writeCount = 0;
    return true;
}

void loggerWrite(const StateEstimate& state,
                 const SensorData&    sensors,
                 const GuidanceState& guidance,
                 FlightPhase          phase,
                 float                actualPositionCm) {
    if (!s_ready) return;

    LogRecord rec;
    rec.timeUs            = state.vertical.timeUs;
    rec.phase             = static_cast<uint8_t>(phase);

    rec.ax                = sensors.ax;
    rec.ay                = sensors.ay;
    rec.az                = sensors.az;
    rec.gx                = sensors.gx;
    rec.gy                = sensors.gy;
    rec.gz                = sensors.gz;

    rec.mx                = sensors.mx;
    rec.my                = sensors.my;
    rec.mz                = sensors.mz;

    rec.hpa               = sensors.hpa;
    rec.tempC             = sensors.tempC;

    rec.h                 = state.vertical.h;
    rec.v                 = state.vertical.v;
    rec.a                 = state.vertical.a;

    rec.q0                = state.attitude.q0;
    rec.q1                = state.attitude.q1;
    rec.q2                = state.attitude.q2;
    rec.q3                = state.attitude.q3;
    rec.tiltDeg           = state.attitude.tiltDeg;

    rec.estimatedApogeeM  = guidance.estimatedApogeeM;
    rec.apogeeErrorM      = guidance.apogeeErrorM;
    rec.targetPositionCm  = guidance.targetPositionCm;

    rec.actualPositionCm  = actualPositionCm;

    file.write(&rec, sizeof(rec));
    s_writeCount++;

    // Periodic flush every 100 records (~2 s at 50 Hz).
    // Keeps data loss window bounded without flushing every write.
    if (s_writeCount % 100 == 0) {
        file.flush();
    }
}

void loggerFlush() {
    if (!s_ready) return;
    file.flush();
}

bool loggerReady() {
    return s_ready;
}
