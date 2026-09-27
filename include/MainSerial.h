#ifndef MAINSERIAL_H
#define MAINSERIAL_H

#include <Arduino.h>


class MainSerial
{
public:

    struct GripperState
    {
        float positionError = 0.0f;
        float torque = 0.0f;

        bool valid = false;

        uint32_t lastRxMs = 0;
    };


    static void begin();

    static void update();


    static void sendTriggerState(
        uint16_t triggerPos,
        uint16_t thumbX,
        uint16_t thumbY
    );


    static const GripperState&
    getGripperState();
};


#endif