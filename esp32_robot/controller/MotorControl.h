/*********************************************************************************
* File name:	MotorControl.hxx
* Author:		VINO
* Version:		V1.0.0
* Date:			2020-06-25 20:57:35 Thursday
* Desc:	
* Others:
*
* History:
*	Date:	Author:
*	Modification:
***********************************************************************************/

#ifndef __MOTOR_CONTROL_HXX__
#define __MOTOR_CONTROL_HXX__

#include "../hwdrivers/Motor.h"

namespace control
{
    class MotorControl
    {
    private:
        hardware::Motor m_left_wheel;
        hardware::Motor m_right_wheel;
    public:
        MotorControl();
        ~MotorControl();

        bool setWheelsPwnCfg(int freq1, int channel1, int res1, int freq2, int channel2, int res2);
        bool initialize(int ain1, int ain2, int pwm1, int bin1, int bin2, int pwm2);

        bool setLinearVelocity(int left, int right);
        bool setAngularVelocity(float w);
    };
} // namespace control

#endif