/*********************************************************************************
* File name:	Motor.hxx
* Author:		VINO
* Version:		V1.0.0
* Date:			2020-06-25 20:31:37 Thursday
* Desc:	
* Others:
*
* History:
*	Date:	Author:
*	Modification:
***********************************************************************************/
#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include "../base/common.h"

#ifdef PLATFORM_ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

namespace hardware
{
    class Motor
    {
    private:
        int m_in1;
        int m_in2;
        int m_pwm_pin;
        int m_freq;
        int m_channel;
        int m_resolution;

    public:
        Motor();
        ~Motor();

    public:
        bool initialize(int in1, int in2);
        bool initialize(int in1, int in2, int pwm);
        bool setPwmConfig(int freq, int channel, int resolution);
        bool setVelocity(int velocity);
    };
} // namespace one_robot

#endif