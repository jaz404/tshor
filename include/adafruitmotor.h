#ifndef ADAFRUITMOTOR_H
#define ADAFRUITMOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <math.h>

class AdafruitMotor
{
public:
    enum class PulseType
    {
        None,
        SoftContact,
        HardContact,
        Slip
    };

    bool begin(TwoWire* wire = &Wire1)
    {
        wire->begin();
        wire->setClock(400000);

        if (!drv.begin(wire))
            return false;

        // Set to false if using an LRA.
        constexpr bool USE_ERM = true;

        if (USE_ERM)
        {
            drv.useERM();
            drv.selectLibrary(1);
        }
        else
        {
            drv.useLRA();
            drv.selectLibrary(6);
        }

        drv.setRealtimeValue(0);
        drv.setMode(DRV2605_MODE_INTTRIG);

        return true;
    }

    // Existing waveform library functionality

    void playEffect(uint8_t effectId, uint32_t durationMs)
    {
        cancelPulse();

        drv.setMode(DRV2605_MODE_INTTRIG);
        rtpActive = false;

        drv.setWaveform(0, effectId);
        drv.setWaveform(1, 0);
        drv.go();

        delay(durationMs);
        drv.stop();
    }

    void vibrateStrong(uint32_t durationMs)
    {
        startRealtime();

        drv.setRealtimeValue(127);
        delay(durationMs);

        drv.setRealtimeValue(0);
        stopRealtime();
    }

    void testStrengths()
    {
        startRealtime();

        const uint8_t strengths[] =
            {20, 40, 60, 80, 100, 127};

        for (uint8_t strength : strengths)
        {
            drv.setRealtimeValue(strength);
            delay(500);

            drv.setRealtimeValue(0);
            delay(500);
        }

        stopRealtime();
    }

    /*
    first will test with custom waveforms but should do a comparision with these and built in ones
    */

    void startRealtime()
    {
        cancelPulse();

        drv.setRealtimeValue(0);
        drv.setMode(DRV2605_MODE_REALTIME);

        rtpActive = true;
        lastStrength = 0;
    }

    void setStrength(uint8_t strength)
    {
        if (!rtpActive)
            startRealtime();

        // Signed RTP: positive amplitudes 0-127
        strength = constrain(strength, 0, 127);

        if (strength != lastStrength)
        {
            drv.setRealtimeValue(strength);
            lastStrength = strength;
        }
    }

    void stopRealtime()
    {
        cancelPulse();

        drv.setRealtimeValue(0);
        drv.setMode(DRV2605_MODE_INTTRIG);

        rtpActive = false;
        lastStrength = 0;
    }

    // -----------------------------------------
    // Non-blocking contact waveforms
    // -----------------------------------------

    void triggerSoftContact(uint8_t peak = 65)
    {
        startPulse(PulseType::SoftContact, peak);
    }

    void triggerHardContact(uint8_t peak = 110)
    {
        startPulse(PulseType::HardContact, peak);
    }

    void triggerSlip(uint8_t peak = 75)
    {
        startPulse(PulseType::Slip, peak);
    }

    // Automatically choose contact waveform
    // from normalized contact strength [0,1].
    void triggerContact(float strength)
    {
        strength = constrain(strength, 0.0f, 1.0f);

        if (strength < 0.5f)
        {
            uint8_t peak = 35 + (uint8_t)(strength * 80);
            triggerSoftContact(peak);
        }
        else
        {
            uint8_t peak = 65 + (uint8_t)(strength * 60);
            triggerHardContact(peak);
        }
    }

    // Call frequently from loop().
    // Does not use delay().
    void update()
    {
        if (activePulse == PulseType::None)
            return;

        const uint32_t now = millis();
        const uint32_t elapsed = now - pulseStartMs;

        constexpr uint32_t UPDATE_PERIOD_MS = 5;

        if (now - lastUpdateMs < UPDATE_PERIOD_MS)
            return;

        lastUpdateMs = now;

        float envelope = 0.0f;
        uint32_t duration = 0;

        switch (activePulse)
        {
        case PulseType::SoftContact:
        {
            duration = 120;

            // 10 ms attack, then exponential decay.
            if (elapsed < 10)
                envelope = elapsed / 10.0f;
            else
                envelope = expf(-(elapsed - 10) / 35.0f);

            break;
        }

        case PulseType::HardContact:
        {
            duration = 85;

            // Fast attack followed by sharp decay.
            if (elapsed < 5)
                envelope = 1.0f;
            else
                envelope = expf(-(elapsed - 5) / 22.0f);

            break;
        }

        case PulseType::Slip:
        {
            duration = 135;

            // Three short pulses, 45 ms apart.
            uint32_t phase = elapsed % 45;

            envelope = expf(-phase / 10.0f);

            break;
        }

        default:
            break;
        }

        if (elapsed >= duration)
        {
            setStrength(0);
            activePulse = PulseType::None;
            return;
        }

        uint8_t amplitude = (uint8_t)constrain(
            peakAmplitude * envelope,
            0.0f,
            127.0f
        );

        setStrength(amplitude);
    }

    bool isPlaying() const
    {
        return activePulse != PulseType::None;
    }

    // -----------------------------------------
    // Gripper-based event detection
    // -----------------------------------------

    void updateContactFeedback(
        float externalTorque,
        float positionError)
    {
        if (!isfinite(externalTorque) ||
            !isfinite(positionError))
        {
            return;
        }

        float torque = fabsf(externalTorque);
        float error = fabsf(positionError);

        // EXAMPLE thresholds. Tune to your sensor.
        // Torque is assumed to be in Nm.
        // Error is assumed to be in revolutions.
        constexpr float TORQUE_ON  = 0.20f;
        constexpr float TORQUE_OFF = 0.10f;

        // Scaling ranges (not detection thresholds).
        constexpr float TORQUE_SCALE = 1.0f;
        constexpr float ERROR_SCALE  = 0.05f;

        // Hysteresis: rearm only after release.
        if (contactActive)
        {
            if (torque < TORQUE_OFF)
                contactActive = false;

            return;
        }

        if (torque < TORQUE_ON)
            return;

        // A new contact was detected.
        contactActive = true;

        const uint32_t now = millis();

        // Prevent repeated buzzing.
        constexpr uint32_t COOLDOWN_MS = 250;

        if (hasTriggered &&
            now - lastContactMs < COOLDOWN_MS)
        {
            return;
        }

        lastContactMs = now;
        hasTriggered = true;

        // Normalize gripper measurements.
        float torqueStrength = constrain(
            torque / TORQUE_SCALE, 0.0f, 1.0f
        );

        float errorStrength = constrain(
            error / ERROR_SCALE, 0.0f, 1.0f
        );

        // Torque contributes more than position error.
        float strength =
            0.75f * torqueStrength +
            0.25f * errorStrength;

        triggerContact(strength);
    }

    void resetContactDetection()
    {
        contactActive = false;
        hasTriggered = false;
        lastContactMs = 0;
    }

    void stop()
    {
        cancelPulse();

        drv.setRealtimeValue(0);
        drv.stop();
        drv.setMode(DRV2605_MODE_INTTRIG);

        rtpActive = false;
        lastStrength = 0;
        resetContactDetection();
    }

private:
    Adafruit_DRV2605 drv;

    bool rtpActive = false;
    uint8_t lastStrength = 0;

    PulseType activePulse = PulseType::None;

    uint8_t peakAmplitude = 0;

    uint32_t pulseStartMs = 0;
    uint32_t lastUpdateMs = 0;

    bool contactActive = false;
    bool hasTriggered = false;
    uint32_t lastContactMs = 0;

    void cancelPulse()
    {
        activePulse = PulseType::None;
    }

    void startPulse(PulseType type, uint8_t peak)
    {
        // Do not interrupt a running haptic event.
        if (isPlaying())
            return;

        if (!rtpActive)
            startRealtime();

        activePulse = type;
        peakAmplitude = constrain(peak, 0, 127);

        pulseStartMs = millis();
        lastUpdateMs = pulseStartMs - 5;

        // Start immediately rather than waiting
        // for the next update.
        setStrength(peakAmplitude);
    }
};

#endif
