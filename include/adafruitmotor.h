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

    void stop()
    {
        drv.stop();
    }

private:
    Adafruit_DRV2605 drv;
};

#endif