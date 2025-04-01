
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
#ifndef _ENCODER_H
#define _ENCODER_H
// DOM-IGNORE-END

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

#define MC_POL_NUM                       7
#define MC_ENCODER_MAX                   65535
#define MC_MECHA_ANGLE_MAX               360.0

#define MC_ELEC_PER_ENCODER              (MC_ENCODER_MAX/MC_POL_NUM)
#define MC_CONVERT_ELECANGLE             (360.0/MC_ELEC_PER_ENCODER)
#define MC_CONVERT_MECHAANGLE            (360.0/MC_ENCODER_MAX)

#define ENCODER_PARAM_ELEC_ANGLE         1
#define ENCODER_PARAM_MECHA_ANGLE        2
#define ENCODER_PARAM_ELEC_OMEGA         3
#define ENCODER_PARAM_MECHA_OMEGA        4

#define ENCODER_CNT_UPPER_THREAD         (MC_ENCODER_MAX*3/4)
#define ENCODER_CNT_LOWER_THREAD         (MC_ENCODER_MAX*1/4)

#define ENCODER_ANGLE_UPPER_THREAD       (MC_MECHA_ANGLE_MAX*3/4)
#define ENCODER_ANGLE_LOWER_THREAD       (MC_MECHA_ANGLE_MAX*1/4)

#define ENCODER_OMEGA_TO_RPM             (60.0/360.0)  /* 60min, 360degree per circle*/
#define ENCODER_CNT_DELTA_LIMIT          65

typedef struct
{
    int32_t  ReadReg;
	int32_t  ReadCurr;
    int32_t  ReadLast;
    int32_t  Cycle;
    int32_t  OrientationOk;
    
    float    RunDirCoeff;
	float    ElecAngle;
	float    MechaAngle;
    float    MechaAngleAcc;
    float    MechaAngleBias;
    float    MechaAngleRelative;
	float    ElecAngleLast;
	float    MechaAngleLast;
	float    ElecOmega;
	float    MechaOmega;
	float    MechaOmegaFilter;
    
    uint32_t DirStudyCode[6];
    uint32_t DirStudyCnt;
}mcParam_Encoder;


extern mcParam_Encoder  gEncoder;

extern void mc_EncoderDirCoeffGet(void);
extern void mc_EncoderStartRead(void);
extern void mc_EncoderInit(void);
extern void mc_EncoderClear(void);
extern void mc_EncoderGetParam(void *Target, uint8_t ParamType);
extern void mc_EncoderPeriodCal(mcParam_Encoder *Encoder, float DeletTime);
extern void mc_EncoderSetOrigin(void);
#ifdef __cplusplus
}
#endif

#endif // #ifndef _MC_LIB_H
/*******************************************************************************
 End of File
*/
