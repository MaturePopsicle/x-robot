/*********************************************************************************
* File name:	MotorControl.cxx
* Author:		VINO
* Version:		V1.0.0
* Date:			2020-06-25 20:57:12 Thursday
* Desc:	
* Others:
*
* History:
*	Date:	Author:
*	Modification:
***********************************************************************************/

#include "MotorControl.h"

using namespace control;

/***********************************************************************************  
* Function:
* Description:
* Input:
* Output:
* Return:
* Others:
*
* @Author:	VINO 
* @History:	[2020-06-25 21:10:00Thursday] Created
***********************************************************************************/
MotorControl::MotorControl()
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
* @History:	[2020-06-25 21:10:03Thursday] Created
***********************************************************************************/
MotorControl::~MotorControl()
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
* @History:	[2020-06-26 11:18:36Friday] Created
***********************************************************************************/
bool MotorControl::setWheelsPwnCfg(int freq1, int channel1, int res1, int freq2, int channel2, int res2)
{
    m_left_wheel.setPwmConfig(freq1, channel1, res1);
    m_right_wheel.setPwmConfig(freq2, channel2, res2);
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
* @History:	[2020-06-26 10:55:07Friday] Created
***********************************************************************************/
bool MotorControl::initialize(int ain1, int ain2, int pwm1, int bin1, int bin2, int pwm2)
{
    m_left_wheel.initialize(ain1, ain2, pwm1);
    m_right_wheel.initialize(bin1, bin2, pwm2);
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
* @History:	[2020-06-25 21:09:56Thursday] Created
***********************************************************************************/
bool MotorControl::setLinearVelocity(int left, int right)
{
    m_left_wheel.setVelocity(left);
    m_right_wheel.setVelocity(right);
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
* @History:	[2020-06-25 21:09:49Thursday] Created
***********************************************************************************/
bool MotorControl::setAngularVelocity(float w)
{
    if(w > M_PI)
    w = w > M_PI ? M_PI : w;
    w = w < -M_PI ? 0 : w;
    return true;
}
