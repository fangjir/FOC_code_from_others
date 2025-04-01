
/*******************************************************************************
  Motor Control Library Interface Header

  Company:
    Microchip Technology Inc. 

  File Name:  
    mc_Lib.h

  Summary:
    Motor Control Library Header File.

  Description:
    This file describes the macros, structures and APIs used by Motor Control Library for SAMD5x 
*******************************************************************************/


// DOM-IGNORE-BEGIN   
/*******************************************************************************
Copyright (c) <2018> released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#ifndef _MCLIB_H
#define _MCLIB_H
// DOM-IGNORE-END

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define     SIN_MASK     0x0C00
#define     ANG_0_90     0x0000
#define     ANG_90_180   0x0400
#define     ANG_180_270  0x0800
#define     ANG_270_360  0x0C00

#define     ANGLE_MAX     ((float)360.0)
#define     ANGLE_STEP1   ((float)(360.0 / 4096.0))

#define     SQRT3        ((float)1.7320508076)        /* Defines value for sqrt(3)   */
#define     DIV_SQRT3    ((float)0.5773502692)        /* Defines value for 1/sqrt(3) */
#define     M_PI         ((float)3.14159265358979323846)
#define     M_PI_2       ((float)1.57079632679489661923)

#define     TOTAL_SINE_TABLE_ANGLE                 ((float)(2 * M_PI))
#define     TABLE_SIZE                              256
#define     ANGLE_STEP2                            ((float)(TOTAL_SINE_TABLE_ANGLE / TABLE_SIZE))
#define     ONE_BY_ANGLE_STEP                      ((float)(1 / ANGLE_STEP2))

#define     CW    1
#define     CCW   -1


typedef struct
{
	float i_a;
	float i_b;
	float i_c;
}mcParam_Curr_abc;

typedef struct
{
	float i_alpha;
	float i_beta;
}mcParam_Curr_alpha_beta;

typedef struct
{
	float i_d;
	float i_q;
}mcParam_Curr_d_q;

typedef struct
{
	float u_alpha;
	float u_beta;
}mcParam_Volt_alpha_beta;

typedef struct
{
	float u_d;
	float u_q;
}mcParam_Volt_d_q;

typedef struct
{
	float sin;
	float cos;
}mcParam_Trigonometric;

typedef struct 
{
    float   PWMPeriod;  // PWM Period in PWM Timer Counts
    float   Vr1;        // Normalized Phase A voltage obtained using modified Clarke transform
    float   Vr2;        // Normalized Phase B voltage obtained using modified Clarke transform
    float   Vr3;        // Normalized Phase C voltage obtained using modified Clarke transform
    float	T1;         // Length of Vector T1
    float   T2;         // Length of Vector T2
    float   Ta;         // Ta = To/2 + T1 + T2
    float   Tb;         // Tb = To/2 + T2
    float   Tc;         // Tc = To/2
    float 	PWM_A;      // Phase A Duty Cycle
    float   PWM_B;      // Phase B Duty Cycle
    float   PWM_C;      // Phase C Duty Cycle
} mcParam_SVPWM;

typedef struct 
{    
    float   Kp;    // Proportional Coefficient of the PI Compensator
    float   Ki;    // Integral Coefficient of the PI Compensator
    float   Kd;
    float   Kc;    // Anti-windup Coefficient of the PI Compensator
	float   Out;   // Proportional + Integral Output of the PI Compensator
	float   Err_Prev;
    float   OutMax;// Max output limit of the PI Compensator
    float   OutMin;// Min output limit of the PI Compensator
    float   Sum;  // Integrator Output of the PI Compensator
} mcParam_PID;


extern void Clarke(mcParam_Curr_abc * Curr_Input, mcParam_Curr_alpha_beta * Curr_Output);

extern void Park(mcParam_Curr_alpha_beta * Curr_Input, mcParam_Trigonometric * Trig, mcParam_Curr_d_q * Curr_Output);

extern void RevPark(mcParam_Volt_d_q* Volt_Input, mcParam_Trigonometric* Trig, mcParam_Volt_alpha_beta * Volt_Output);

extern void Svpwm(mcParam_Volt_alpha_beta * const Volt_Input, mcParam_SVPWM * const svm);

extern void Pid(float inRef, float inMeas, mcParam_PID * const pParm);

extern void TrigonometricCalc(float Angle, mcParam_Trigonometric * Trig);

extern void LowPassFilter(float *In, float *Out, float coeff);
#ifdef __cplusplus
}
#endif

#endif // #ifndef _MC_LIB_H
/*******************************************************************************
 End of File
*/
