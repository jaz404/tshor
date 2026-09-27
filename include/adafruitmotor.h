#ifndef ADAFRUITMOTOR_H
#define ADAFRUITMOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>

class AdafruitMotor
{
public:
    bool begin(TwoWire* wire = &Wire1)
    {
        wire->begin();
        wire->setClock(400000);

        if (!drv.begin(wire))
        {
            return false;
        }

        // ERM motor
        drv.selectLibrary(1);
        drv.useERM();

        // Internal trigger mode for waveform effects
        drv.setMode(DRV2605_MODE_INTTRIG);

        return true;
    }

    void playEffect(uint8_t effectId, uint32_t durationMs)
    {
        drv.setWaveform(0, effectId);
        drv.setWaveform(1, 0);  // End sequence

        drv.go();

        delay(durationMs);

        drv.stop();
    }
    
    // more modes to try with
    void vibrateStrong(uint32_t durationMs)
    {
        drv.setMode(DRV2605_MODE_REALTIME);

        drv.setRealtimeValue(127);   // strong positive drive
        delay(durationMs);

        drv.setRealtimeValue(0);

        drv.setMode(DRV2605_MODE_INTTRIG);
    }
    void testStrengths()
    {
        drv.setMode(DRV2605_MODE_REALTIME);

        const uint8_t strengths[] = {20, 40, 60, 80, 100, 127};

        for (int i = 0; i < 6; i++)
        {
            drv.setRealtimeValue(strengths[i]);

            delay(500);

            drv.setRealtimeValue(0);
            delay(500);
        }

        drv.setRealtimeValue(0);
        drv.setMode(DRV2605_MODE_INTTRIG);
    }
    // void stop()
    // {
    //     drv.stop();
    // }
// -------------------------------------------------------
    // Continuous haptic feedback
    // -------------------------------------------------------

    void startRealtime()
    {
        Serial.println("[DRV] Entering RTP mode");

        drv.setMode(DRV2605_MODE_REALTIME);
        drv.setRealtimeValue(0);
    }


    void setStrength(uint8_t strength)
    {
        strength = constrain(strength, 0, 127);

        drv.setRealtimeValue(strength);
    }


    void stopRealtime()
    {
        Serial.println("[DRV] Stopping RTP");

        drv.setRealtimeValue(0);

        drv.setMode(DRV2605_MODE_INTTRIG);
    }


    void stop()
    {
        drv.setRealtimeValue(0);
        drv.stop();
    }

private:
    Adafruit_DRV2605 drv;
};

#endif
