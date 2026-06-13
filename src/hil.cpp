#ifdef HIL_MODE

#include "hil.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// --- Packet payloads ----------------------------------------------------

struct __attribute__((packed)) InitPacket {
    float basePressureHpa;
    float baseTempC;
};

struct __attribute__((packed)) SensorPacket {
    uint32_t simTimeUs;
    float ax, ay, az;
    float gx, gy, gz;
    float hpa, tempC;
    float mx, my, mz;
    uint8_t flags;
};

struct __attribute__((packed)) StatusPacket {
    uint32_t simTimeUs;
    uint8_t  phase;
    float estH, estV, tiltDeg;
    float estApogeeM, apogeeErrM;
    float targetPosCm, actualPosCm;
    float kfP00, kfP11;
    float actDutyPct;
};

// --- Module state ---------------------------------------------------------

static InitPacket   s_init       = { SEALEVEL_HPA, 15.0f };
static SensorPacket s_sensor     = {};
static bool         s_baroFresh  = false;
static bool         s_magFresh   = false;
static uint32_t     s_simTimeUs  = 0;

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// --- Receive parser --------------------------------------------------------

enum class RxState : uint8_t { SYNC1, SYNC2, TYPE, LEN, PAYLOAD, CRC };

static RxState  s_rxState = RxState::SYNC1;
static uint8_t  s_rxType  = 0;
static uint8_t  s_rxLen   = 0;
static uint8_t  s_rxPos   = 0;
static uint8_t  s_rxBuf[64];

// Feed one byte; returns completed packet type or 0.
static uint8_t rxFeed(uint8_t byte) {
    switch (s_rxState) {
        case RxState::SYNC1:
            if (byte == HIL_SYNC1) s_rxState = RxState::SYNC2;
            break;
        case RxState::SYNC2:
            s_rxState = (byte == HIL_SYNC2) ? RxState::TYPE : RxState::SYNC1;
            break;
        case RxState::TYPE:
            s_rxType  = byte;
            s_rxState = RxState::LEN;
            break;
        case RxState::LEN:
            if (byte > sizeof(s_rxBuf)) { s_rxState = RxState::SYNC1; break; }
            s_rxLen   = byte;
            s_rxPos   = 0;
            s_rxState = (byte == 0) ? RxState::CRC : RxState::PAYLOAD;
            break;
        case RxState::PAYLOAD:
            s_rxBuf[s_rxPos++] = byte;
            if (s_rxPos >= s_rxLen) s_rxState = RxState::CRC;
            break;
        case RxState::CRC: {
            s_rxState = RxState::SYNC1;
            uint8_t full[2 + sizeof(s_rxBuf)];
            full[0] = s_rxType;
            full[1] = s_rxLen;
            memcpy(&full[2], s_rxBuf, s_rxLen);
            if (crc8(full, 2 + s_rxLen) == byte) return s_rxType;
            break;
        }
    }
    return 0;
}

static void sendPacket(uint8_t type, const void* payload, uint8_t len) {
    uint8_t frame[4 + sizeof(StatusPacket) + 1];
    frame[0] = HIL_SYNC1;
    frame[1] = HIL_SYNC2;
    frame[2] = type;
    frame[3] = len;
    memcpy(&frame[4], payload, len);
    frame[4 + len] = crc8(&frame[2], 2 + len);
    Serial.write(frame, 5 + len);
    Serial.send_now();
}

// Handle a completed packet; returns true if it was a SENSOR packet.
static bool handlePacket(uint8_t type) {
    switch (type) {
        case HIL_PKT_INIT: {
            memcpy(&s_init, s_rxBuf, sizeof(s_init));
            uint8_t version = HIL_PROTOCOL_VERSION;
            sendPacket(HIL_PKT_INIT_ACK, &version, 1);
            return false;
        }
        case HIL_PKT_SENSOR:
            memcpy(&s_sensor, s_rxBuf, sizeof(s_sensor));
            s_simTimeUs = s_sensor.simTimeUs;
            if (s_sensor.flags & HIL_FLAG_BARO) s_baroFresh = true;
            if (s_sensor.flags & HIL_FLAG_MAG)  s_magFresh  = true;
            return true;
        default:
            return false;
    }
}

// --- Public API -------------------------------------------------------------

void hilInit() {
    Serial.println("HIL: waiting for INIT from host...");
    uint32_t lastBlinkMs = 0;
    while (true) {
        while (Serial.available()) {
            uint8_t type = rxFeed((uint8_t)Serial.read());
            if (type == HIL_PKT_INIT) {
                handlePacket(type);
                Serial.print("HIL: INIT ok, base pressure ");
                Serial.print(s_init.basePressureHpa);
                Serial.println(" hPa");
                return;
            }
        }
        if (millis() - lastBlinkMs >= 250) {
            lastBlinkMs = millis();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}

bool hilPoll() {
    while (Serial.available()) {
        uint8_t type = rxFeed((uint8_t)Serial.read());
        if (type != 0 && handlePacket(type)) return true;
    }
    return false;
}

uint32_t hilTimeUs()         { return s_simTimeUs; }
float hilBasePressureHpa()   { return s_init.basePressureHpa; }
float hilBaseTempC()         { return s_init.baseTempC; }

void hilReadIMU(SensorData& d) {
    d.ax = s_sensor.ax;
    d.ay = s_sensor.ay;
    d.az = s_sensor.az;
    d.gx = s_sensor.gx;
    d.gy = s_sensor.gy;
    d.gz = s_sensor.gz;
    d.imuTimeUs  = s_sensor.simTimeUs;
    d.imuUpdated = (s_sensor.flags & HIL_FLAG_IMU) != 0;
}

void hilReadBaro(SensorData& d) {
    if (!s_baroFresh) { d.baroUpdated = false; return; }
    d.hpa        = s_sensor.hpa;
    d.tempC      = s_sensor.tempC;
    d.baroTimeUs = s_sensor.simTimeUs;
    d.baroUpdated = true;
    s_baroFresh   = false;
}

void hilReadMag(SensorData& d) {
    if (!s_magFresh) { d.magUpdated = false; return; }
    d.mx        = s_sensor.mx;
    d.my        = s_sensor.my;
    d.mz        = s_sensor.mz;
    d.magTimeUs = s_sensor.simTimeUs;
    d.magUpdated = true;
    s_magFresh   = false;
}

void hilSendStatus(const StateEstimate& state,
                   const GuidanceState& guidance,
                   FlightPhase          phase,
                   float                actualPosCm,
                   float                kfP00,
                   float                kfP11,
                   float                actDutyPct) {
    StatusPacket p;
    p.simTimeUs   = s_simTimeUs;
    p.phase       = static_cast<uint8_t>(phase);
    p.estH        = state.vertical.h;
    p.estV        = state.vertical.v;
    p.tiltDeg     = state.attitude.tiltDeg;
    p.estApogeeM  = guidance.estimatedApogeeM;
    p.apogeeErrM  = guidance.apogeeErrorM;
    p.targetPosCm = guidance.targetPositionCm;
    p.actualPosCm = actualPosCm;
    p.kfP00       = kfP00;
    p.kfP11       = kfP11;
    p.actDutyPct  = actDutyPct;
    sendPacket(HIL_PKT_STATUS, &p, sizeof(p));
}

#endif // HIL_MODE
