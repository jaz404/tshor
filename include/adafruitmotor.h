#ifndef ADAFRUITMOTOR_H
#define ADAFRUITMOTOR_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <math.h>

class AdafruitMotor
{
public:
    bool begin(TwoWire* wire = &Wire1)
    {
        wire->begin(); wire->setClock(400000);
        ready = drv.begin(wire);
        if (!ready) return false;
        drv.useERM(); drv.selectLibrary(1);
        drv.setRealtimeValue(0); lastStrength = 0;
        drv.setMode(DRV2605_MODE_REALTIME);
        stop();
        return true;
    }

    // Call every loop. Torque is the deadband-subtracted estimate in N m;
    // positionError is in revolutions. sampleMs uses this MCU's millis() clock.
    void updateContactFeedback(float externalTorque, float positionError,
                               uint32_t sampleMs, bool valid = true)
    {
        if (!ready) return;
        const uint32_t now = millis();
        if (!valid || !isfinite(externalTorque) || !isfinite(positionError) ||
            now - sampleMs >= TIMEOUT_MS)
        {
            stop(); return;
        }
        if (playing && now - pulseStartMs >= PULSE_MS)
        {
            playing = false; writeStrength(0);
        }
        const float magnitude = fabsf(externalTorque);
        if (!initialized) { seed(magnitude, positionError, sampleMs); return; }
        if (sampleMs == lastSampleMs) return;
        const uint32_t deltaMs = sampleMs - lastSampleMs;
        if (deltaMs >= TIMEOUT_MS)
        {
            stop(); seed(magnitude, positionError, sampleMs); return;
        }
        lastSampleMs = sampleMs;
        if (deltaMs > MAX_CONFIRM_GAP_MS)
        {
            contactPending = releasePending = false;
            // Missing samples cannot establish a new rise or valley.
            peakSignal = peak = valley = magnitude;
            waitingForRise = false;
        }
        const float dt = deltaMs * 0.001f;
        filteredTorque += dt / (TORQUE_FILTER_TAU_S + dt) * (magnitude - filteredTorque);
        peakSignal += dt / (PEAK_FILTER_TAU_S + dt) * (magnitude - peakSignal);
        filteredError += dt / (ERROR_FILTER_TAU_S + dt) * (positionError - filteredError);

        if (filteredTorque <= TORQUE_OFF)
        {
            if (!releasePending) { releasePending = true; releaseStartMs = sampleMs; }
            if (sampleMs - releaseStartMs >= RELEASE_MS)
            {
                armed = true; contactActive = false;
                contactPending = false; playing = false;
                writeStrength(0);
            }
        }
        else releasePending = false;

        if (!contactActive)
        {
            if (!armed || magnitude < TORQUE_ON) { contactPending = false; return; }
            if (!contactPending) { contactPending = true; contactStartMs = sampleMs; return; }
            if (sampleMs - contactStartMs < CONTACT_CONFIRM_MS) return;
            contactPending = false; armed = false; contactActive = true;
            peak = valley = peakSignal; waitingForRise = false;
            triggerPulse(now); // first confirmed contact
        }
        else
        {
            // Retrigger on a significant rise AFTER a significant fall.
            // A steady load never generates periodic pulses by itself.
            if (!waitingForRise)
            {
                if (peakSignal > peak) peak = peakSignal;
                if (peak - peakSignal >= PEAK_FALL_NM)
                {
                    waitingForRise = true; valley = peakSignal;
                }
            }
            else
            {
                if (peakSignal < valley) valley = peakSignal;
                if (peakSignal >= TORQUE_ON && peakSignal - valley >= PEAK_RISE_NM)
                {
                    waitingForRise = false; peak = peakSignal;
                    triggerPulse(now);
                }
            }
        }
        if (playing)
        {
            // Let strength grow with this peak, without extending pulse duration.
            const uint8_t amplitude = positionStrength();
            if (amplitude > lastStrength) writeStrength(amplitude);
        }
    }

    float getGatedPositionError() const
    {
        return contactActive && fabsf(filteredError) > ERROR_DEADBAND ? filteredError : 0.0f;
    }

    void stop()
    {
        if (ready) writeStrength(0);
        initialized = armed = contactActive = playing = hasTriggered = false;
        contactPending = releasePending = waitingForRise = false;
        filteredTorque = filteredError = peakSignal = peak = valley = 0.0f;
    }

private:
    static constexpr float TORQUE_FILTER_TAU_S = 0.021f;
    static constexpr float TORQUE_ON = 0.006f;
    static constexpr float TORQUE_OFF = 0.003f;
    static constexpr float ERROR_FILTER_TAU_S = 0.010f;
    static constexpr float ERROR_DEADBAND = 0.002f;
    static constexpr float ERROR_FULL = 0.040f;

    // Separate faster torque filter for peaks inside an established contact.
    static constexpr float PEAK_FILTER_TAU_S = 0.005f;
    static constexpr float PEAK_RISE_NM = 0.008f;
    static constexpr float PEAK_FALL_NM = 0.006f;
    static constexpr uint8_t MIN_STRENGTH = 60;
    static constexpr uint8_t MAX_STRENGTH = 80;
    static constexpr uint32_t CONTACT_CONFIRM_MS = 35;
    static constexpr uint32_t MAX_CONFIRM_GAP_MS = 30;
    static constexpr uint32_t PULSE_MS = 30;
    static constexpr uint32_t MIN_INTERVAL_MS = 60; // start-to-start, at least 30ms off
    static constexpr uint32_t RELEASE_MS = 40;
    static constexpr uint32_t TIMEOUT_MS = 100;

    Adafruit_DRV2605 drv;
    bool ready = false, initialized = false, armed = false;
    bool contactActive = false, playing = false, hasTriggered = false;
    bool contactPending = false, releasePending = false, waitingForRise = false;
    float filteredTorque = 0, filteredError = 0, peakSignal = 0, peak = 0, valley = 0;
    uint32_t lastSampleMs = 0, pulseStartMs = 0, contactStartMs = 0, releaseStartMs = 0;
    uint8_t lastStrength = 0;

    void seed(float torque, float error, uint32_t sampleMs)
    {
        filteredTorque = peakSignal = peak = valley = torque;
        filteredError = error; lastSampleMs = sampleMs; initialized = true;
        // Observe release before enabling feedback after startup or a timeout.
        armed = false; contactPending = false;
        releasePending = torque <= TORQUE_OFF; releaseStartMs = sampleMs;
    }
    uint8_t positionStrength() const
    {
        const float scale = constrain((fabsf(filteredError) - ERROR_DEADBAND) /
                                      (ERROR_FULL - ERROR_DEADBAND), 0.0f, 1.0f);
        return scale <= 0 ? 0 : (uint8_t)(MIN_STRENGTH +
                              (MAX_STRENGTH - MIN_STRENGTH) * scale + 0.5f);
    }
    void triggerPulse(uint32_t now)
    {
        if (playing || (hasTriggered && now - pulseStartMs < MIN_INTERVAL_MS)) return;
        const uint8_t amplitude = positionStrength();
        if (!amplitude) return;
        playing = hasTriggered = true; pulseStartMs = now;
        writeStrength(amplitude);
        // Peaks arriving during cooldown are discarded, never replayed later.
    }
    void writeStrength(uint8_t value)
    {
        if (value != lastStrength) { drv.setRealtimeValue(value); lastStrength = value; }
    }
};
#endif
