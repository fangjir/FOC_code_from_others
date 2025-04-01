/*******************************************************************************
 Motor Control Library Filee - PLL Estimator

  Company:
    Microchip Technology Inc.

  File Name:
    mclib_generic_float.c

  Summary:
    This file contains the motor control algorithm functions.

  Description:
    This file contains the motor control algorithm functions like clarke transform,
    park transform. This library is implemented with float data type. 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END
#include "main.h"
#include "mc_Lib.h"
#include "mc_Encoder.h"
#include <math.h>

extern SPI_HandleTypeDef hspi1;
mcParam_Encoder  gEncoder;
float * pEncoder;

void mc_EncoderInit(void)
{
    gEncoder.MechaAngleAcc  = 0;
    gEncoder.MechaAngle     = 0;
    gEncoder.MechaAngleBias = 0;
    gEncoder.Cycle          = 0;
}

void mc_EncoderClear(void)
{
	gEncoder.ElecAngleLast  = gEncoder.ElecAngle;
	gEncoder.MechaAngleLast = gEncoder.MechaAngleBias;
	gEncoder.ElecOmega  = 0;
	gEncoder.MechaOmega = 0;
}


void mc_EncoderPeriodCal(mcParam_Encoder *Encoder, float DeletTime)
{
    float Angle;
    static uint8_t SyncFirst = 0;
    
    if(Encoder->RunDirCoeff < 0)
        Encoder->ReadCurr = Encoder->ReadReg;
    else
        Encoder->ReadCurr = 65536 - Encoder->ReadReg;
    
    if(0 == SyncFirst)
    {
        SyncFirst = 1;
        Encoder->ReadLast = Encoder->ReadCurr;
    }
    if((Encoder->ReadCurr < ENCODER_CNT_LOWER_THREAD)&&(Encoder->ReadLast > ENCODER_CNT_UPPER_THREAD))
    {
        Angle = Encoder->ReadCurr + MC_ENCODER_MAX - Encoder->ReadLast;
        Encoder->Cycle++;
    }
    else if((Encoder->ReadCurr > ENCODER_CNT_UPPER_THREAD)&&(Encoder->ReadLast < ENCODER_CNT_LOWER_THREAD))
    {
        Angle = Encoder->ReadCurr - MC_ENCODER_MAX - Encoder->ReadLast;
        Encoder->Cycle--;
    }
    else
    {
        Angle = Encoder->ReadCurr - Encoder->ReadLast;
    }
    Encoder->ReadLast = Encoder->ReadCurr;
    
    if(0 == DeletTime)
    {
        Encoder->MechaOmega = 0;
    }
    else
    {
        Encoder->MechaOmega = (float)(60.0f / 65535.0f) * Angle / DeletTime;
    }
    
    LowPassFilter(&(Encoder->MechaOmega), &(Encoder->MechaOmegaFilter), 0.2f);
    
    Angle = Encoder->ReadCurr;
    if(Angle < 0)
    {
        Angle += MC_ENCODER_MAX;
    }
    Encoder->MechaAngle          = Angle * MC_CONVERT_MECHAANGLE;
    Encoder->MechaAngleAcc       = Encoder->MechaAngle + (Encoder->Cycle * 360);
    if(Encoder->OrientationOk)
        Encoder->MechaAngleRelative  =  Encoder->MechaAngleAcc  - Encoder->MechaAngleBias;
    else
        Encoder->MechaAngleRelative = 0;
}

/**
  * @brief  start spi read encoder chip
  */
void mc_EncoderStartRead(void)
{
    uint32_t Cmd = 0x0000;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)(&Cmd), (uint8_t *)(&gEncoder.ReadReg), 1, 100);
}

/**
  * @brief 保存初始位子编码器的值
  */
void mc_EncoderSetOrigin(void)
{	    
    gEncoder.MechaAngleAcc  = gEncoder.MechaAngle + (gEncoder.Cycle * 360);
    gEncoder.MechaAngleBias = gEncoder.MechaAngleAcc;
    gEncoder.MechaAngleRelative= 0;
	gEncoder.ElecOmega      = 0;
	gEncoder.MechaOmega     = 0;
}

/**
  * @brief  Read encoder param
  * @input  Type param type
  *   @arg ENCODER_PARAM_ELEC_ANGLE         ElecAngle
  *   @arg ENCODER_PARAM_MECHA_ANGLE        MechaAngle
  *   @arg ENCODER_PARAM_ELEC_ANGLE_LAST    ElecAngleLast
  *   @arg ENCODER_PARAM_MECHA_ANGLE_LAST   MechaAngleLast
  *   @arg ENCODER_PARAM_ELEC_OMEGA         ElecOmega
  *   @arg ENCODER_PARAM_MECHA_OMEGA        MechaOmega
  */
void mc_EncoderGetParam(void *Target, uint8_t ParamType)
{
	switch(ParamType)
	{
		case ENCODER_PARAM_ELEC_ANGLE:
			break;
			
		case ENCODER_PARAM_MECHA_ANGLE:
			*((float *)Target) = gEncoder.MechaAngleRelative;
			break;

		case ENCODER_PARAM_ELEC_OMEGA:
			break;
			
		case ENCODER_PARAM_MECHA_OMEGA:
			*((float *)Target) = gEncoder.MechaOmegaFilter;
			break;		
		
		default:
			break;
	}
}

void mc_EncoderDirCoeffGet(void)
{
    int32_t i,j;
    
    if(0 == gEncoder.RunDirCoeff)
    {
        gEncoder.DirStudyCode[gEncoder.DirStudyCnt++] = gEncoder.ReadCurr;
        if(gEncoder.DirStudyCnt >= 6)
        {
            for(i=0,j=0; i<5; i++)
            {
                if(gEncoder.DirStudyCode[i] < gEncoder.DirStudyCode[i+1])
                    j++;
                else
                    j--;
            }
            
            if(j > 0)
                gEncoder.RunDirCoeff = 1.0f;
            else
                gEncoder.RunDirCoeff = -1.0f;
        }
    }
}



