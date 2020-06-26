/*********************************************************************************
* File name:	Motor.cxx
* Author:		VINO
* Version:		V1.0.0
* Date:			2020-06-25 20:31:10 Thursday
* Desc:	
* Others:
*
* History:
*	Date:	Author:
*	Modification:
***********************************************************************************/
#include "Motor.h"

using namespace one_robot;


/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:06:14Friday] Created
***********************************************************************************/
Motor::Motor() : m_in1(AIN1),
                 m_in2(AIN2),
                 m_pwm(PWM1),
                 m_freq(10 * 1000),
                 m_channel(0),
                 m_resolution(8)
{
}

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:06:17Friday] Created
***********************************************************************************/
Motor::~Motor()
{
}

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:06:20Friday] Created
***********************************************************************************/
bool Motor::initialize(int in1, int in2)
{
    m_in1 = in1;
    m_in2 = in2;

    pinMode(m_in1, OUTPUT);
    pinMode(m_in2, OUTPUT);
    digitalWrite(m_in1, HIGH);
    digitalWrite(m_in2, LOW);

    return true;
}

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:06:23Friday] Created
***********************************************************************************/
bool Motor::initialize(int in1, int in2, int pwm_pin)
{
    m_in1 = in1;
    m_in2 = in2;
    m_pwm_pin = pwm_pin;

    ledcSetup(m_channel, m_freq, m_resolution);
    ledcAttachPin(m_pwm_pin, m_channel);

    pinMode(m_in1, OUTPUT);
    pinMode(m_in2, OUTPUT);
    digitalWrite(m_in1, HIGH);
    digitalWrite(m_in2, LOW);

    return true;
}

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:06:25Friday] Created
***********************************************************************************/
bool Motor::setPwmConfig(int freq, int channel, int resolution)
{
    Serial.print("setPwmConfig: ");
    Serial.print(freq);
    m_freq = freq;
    m_channel = channel;
    m_resolution = resolution;
    return true;
}

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-26 11:21:00Friday] Created
***********************************************************************************/
bool Motor::setVelocity(int velocity)
{
    Serial.print("setVelocity: ");
    Serial.println(velocity);
    if(velocity < 0)
    {
        velocity = 0;
    }
    if(velocity >= 255)
    {
        velocity = 255;
    }
    ledcWrite(m_channel, velocity);
    return true;
}