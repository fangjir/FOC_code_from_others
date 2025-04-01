
#include "main.h" 
#include "MotorControl.h"
#include "mc_Lib.h"
#include "mc_Encoder.h"
#include "mc_Hall.h"
#include "mc_UserParam.h"
#include "Cmd.h"
#include <stdio.h>
#include <string.h>

#define FILTER_COEFF_D_Q  0.9f

mcParam_Curr_abc          gCurr_abc;
mcParam_Curr_abc          gCurr_abcBias;
mcParam_Curr_abc          gCurr_abcAD;

mcParam_Curr_alpha_beta   gCurr_alpha_beta;
mcParam_Curr_d_q          gCurr_d_q;
mcParam_Curr_d_q          gCurr_d_q_filter;
mcParam_Volt_alpha_beta   gVolt_alpha_beta;
mcParam_Volt_d_q          gVolt_d_q;
mcParam_Trigonometric     gTrig;
mcParam_SVPWM             gSvpwm;

mcParam_PID               gPID_IQ;
mcParam_PID               gPID_ID;
mcParam_PID               gPID_Speed;
mcParam_PID               gPID_Position;

mcParam_FOC               gFoc;

extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
uint8_t DebugFifo[300];
uint8_t gTitle[] = "\r\n=====================  Motor control start �� ====================\r\n";

void mc_ParamInit(void)
{
	gSvpwm.PWMPeriod = PWM_PERIOD;
    
	gPID_IQ.Kp     = Q_CURRCNTR_PTERM;
	gPID_IQ.Ki     = Q_CURRCNTR_ITERM;
    gPID_IQ.Kd     = 0;
	gPID_IQ.Kc     = 0;//Q_CURRCNTR_CTERM;
	gPID_IQ.OutMax = Q_CURRCNTR_OUTMAX;
	gPID_IQ.OutMin = -Q_CURRCNTR_OUTMAX;
	gPID_IQ.Sum    = 0;
	gPID_IQ.Out    = 0;

	gPID_ID.Kp     = D_CURRCNTR_PTERM;
	gPID_ID.Ki     = D_CURRCNTR_ITERM;
    gPID_ID.Kd     = 0;
	gPID_ID.Kc     = 0;//D_CURRCNTR_CTERM;
	gPID_ID.OutMax = D_CURRCNTR_OUTMAX;
	gPID_ID.OutMin = -D_CURRCNTR_OUTMAX;
	gPID_ID.Sum    = 0;
	gPID_ID.Out    = 0;

	gPID_Speed.Kp      = SPEEDCNTR_PTERM;
	gPID_Speed.Ki      = SPEEDCNTR_ITERM;
    gPID_Speed.Kd      = SPEEDCNTR_DTERM;
	gPID_Speed.Kc      = 0;
	gPID_Speed.OutMax  = SPEEDCNTR_OUTMAX;
	gPID_Speed.OutMin  = -SPEEDCNTR_OUTMAX;
	gPID_Speed.Sum     = 0;
	gPID_Speed.Out     = 0;

	gPID_Position.Kp      = POSCNTR_PTERM;
	gPID_Position.Ki      = POSCNTR_ITERM;
    gPID_Position.Kd      = POSCNTR_DTERM;
	gPID_Position.Kc      = 0;
	gPID_Position.OutMax  = POSCNTR_OUTMAX;
	gPID_Position.OutMin  = -POSCNTR_OUTMAX;
	gPID_Position.Sum     = 0;
	gPID_Position.Out     = 0;

	gFoc.OrientaCnt = 0;
    gFoc.MechaAngleRef = 0;
    gFoc.TorqueCoeff = 1.0;
}

void mc_ParamDefault(void)
{
    gPID_IQ.Kp     = Q_CURRCNTR_PTERM;
	gPID_IQ.Ki     = Q_CURRCNTR_ITERM;
    gPID_IQ.OutMax = Q_CURRCNTR_OUTMAX;
	gPID_IQ.OutMin = -Q_CURRCNTR_OUTMAX;
    gPID_ID.Kp     = D_CURRCNTR_PTERM;
	gPID_ID.Ki     = D_CURRCNTR_ITERM;
    gPID_ID.OutMax = D_CURRCNTR_OUTMAX;
	gPID_ID.OutMin = -D_CURRCNTR_OUTMAX;
    gPID_Speed.Kp      = SPEEDCNTR_PTERM;
	gPID_Speed.Ki      = SPEEDCNTR_ITERM;
    gPID_Speed.OutMax  = SPEEDCNTR_OUTMAX;
	gPID_Speed.OutMin  = -SPEEDCNTR_OUTMAX;
    gPID_Position.Kp      = POSCNTR_PTERM;
	gPID_Position.Ki      = POSCNTR_ITERM;
    gPID_Position.Kd      = POSCNTR_DTERM;
    gPID_Position.OutMax  = POSCNTR_OUTMAX;
	gPID_Position.OutMin  = -POSCNTR_OUTMAX;
    gFoc.TorqueCoeff = 1.0;
}

/**
  * @brief driver enable or disable
  */
void mc_OnOffCtr(uint8_t Status)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, Status);
}

/**
  * @brief pwm duty update
  */
extern TIM_HandleTypeDef htim1;
void mc_PwmLoad(uint32_t pwmA, uint32_t pwmB, uint32_t pwmC)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmA);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmB);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmC);
}

/**
  * @brief read current bias
  */
void mc_CurrentBiasRead(mcParam_Curr_abc * Curr_abc_bias)
{
	static uint32_t Cnt = 0;
	static uint32_t a_Bias = 0, b_Bias = 0, c_Bias = 0;
	uint32_t i, delta_Bias = 0, status = 0;
	
	if(gFoc.OffsetGetOk)
		return;
	
	
    mc_OnOffCtr(0);
	a_Bias += (int32_t)(ADC1->JDR1);
	b_Bias += (int32_t)(ADC1->JDR2);
	c_Bias += (int32_t)(ADC1->JDR3);

	Cnt++;
	if(Cnt >= 64)
	{
		a_Bias >>= 6;
		b_Bias >>= 6;
		c_Bias >>= 6;
		
		if(a_Bias > MC_BIAS_IDEAL)
			delta_Bias = a_Bias - MC_BIAS_IDEAL;
		else
			delta_Bias = MC_BIAS_IDEAL - a_Bias;
		
		if(delta_Bias > MC_BIAS_ERR_MAX)
			status = 1;

		if(b_Bias > MC_BIAS_IDEAL)
			delta_Bias = b_Bias - MC_BIAS_IDEAL;
		else
			delta_Bias = MC_BIAS_IDEAL - b_Bias;
		
		if(delta_Bias > MC_BIAS_ERR_MAX)
			status = 1;
		
		if(c_Bias > MC_BIAS_IDEAL)
			delta_Bias = c_Bias - MC_BIAS_IDEAL;
		else
			delta_Bias = MC_BIAS_IDEAL - c_Bias;
		
		if(delta_Bias > MC_BIAS_ERR_MAX)
			status = 1;
		
		if(0 == status)
		{
			Curr_abc_bias->i_a = a_Bias;
			Curr_abc_bias->i_b = b_Bias;
			Curr_abc_bias->i_c = c_Bias;
		}
		else
		{
			Curr_abc_bias->i_a = MC_BIAS_IDEAL;
			Curr_abc_bias->i_b = MC_BIAS_IDEAL;
			Curr_abc_bias->i_c = MC_BIAS_IDEAL;
		}
			gFoc.OffsetGetOk = 1;
	}
}

void mc_GetCurrentAd(mcParam_Curr_abc * Curr_abc)
{
	float CurrA = 0, CurrB = 0, CurrC = 0, Vbus = 0;

	CurrA = ADC1->JDR1;
	CurrB = ADC1->JDR2;
	CurrC = ADC1->JDR3;
	Vbus  = ADC1->JDR4;
	
	Curr_abc->i_a = CurrA;
	Curr_abc->i_b = CurrB;
	Curr_abc->i_c = CurrC;
}

mcParam_Curr_abc CurrFilter;
void mc_CurrentMeasurement(mcParam_Curr_abc * Curr_abc, mcParam_Curr_abc * Curr_abc_bias)
{
	float CurrA = 0, CurrB = 0, CurrC = 0, Vbus = 0;

	CurrA = ADC1->JDR1;
	CurrB = ADC1->JDR2;
	CurrC = ADC1->JDR3;
	Vbus  = ADC1->JDR4;
    
    gCurr_abcAD.i_a = CurrA;
    gCurr_abcAD.i_b = CurrB;
    gCurr_abcAD.i_c = CurrC;
    
#if 0
	Curr_abc->i_a = (Curr_abc_bias->i_a - CurrA) * MC_CURR_CONV_COEFF2;
	Curr_abc->i_b = (Curr_abc_bias->i_b - CurrB) * MC_CURR_CONV_COEFF2;
	Curr_abc->i_c = (Curr_abc_bias->i_c - CurrC) * MC_CURR_CONV_COEFF2;
#else
    
	CurrA = MC_CURR_CONV_COEFF * (Curr_abc_bias->i_a - CurrA);
	CurrB = MC_CURR_CONV_COEFF * (Curr_abc_bias->i_b - CurrB);
	CurrC = MC_CURR_CONV_COEFF * (Curr_abc_bias->i_c - CurrC);
    
#endif
    
    CurrFilter.i_a = CurrFilter.i_a * MC_CURR_FILTER_COEFF + CurrA * (1 - MC_CURR_FILTER_COEFF);   //低通滤波
    CurrFilter.i_b = CurrFilter.i_b * MC_CURR_FILTER_COEFF + CurrB * (1 - MC_CURR_FILTER_COEFF);
    CurrFilter.i_c = CurrFilter.i_c * MC_CURR_FILTER_COEFF + CurrC * (1 - MC_CURR_FILTER_COEFF);	
    
    Curr_abc->i_a =  CurrFilter.i_a;
    Curr_abc->i_b =  CurrFilter.i_b;
    Curr_abc->i_c =  CurrFilter.i_c;
}


extern DAC_HandleTypeDef hdac1;
uint16_t gDac;
void mc_CoreAlgorithm(ADC_HandleTypeDef *hadc)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);
    /*read hall angle*/
    mc_HallAngleCal(&gHall);
    
	switch(gFoc.RunStep)
    {
        case MC_STATE_BIAS:   //读取电流和电压偏差值
            mc_CurrentBiasRead(&gCurr_abcBias);
            break;
            
        case MC_STATE_HALL_STUDY:
        case MC_STATE_RUN:
        case MC_STATE_USER:	
            /*current read*/
        
            mc_CurrentMeasurement(&gCurr_abc, &gCurr_abcBias);  //读取电流

            /*Trigonometric Calculate*/
            if(MC_STATE_RUN == gFoc.RunStep)  //6
            {
                mc_HallGetParam(&gFoc.Angle,HALL_ACC_ANGLE);
            }
            
            /*sin cos*/
            TrigonometricCalc(gFoc.Angle, &gTrig);
            
            /*Clark*/
            Clarke(&gCurr_abc, &gCurr_alpha_beta);
            
            /*Park*/
            Park(&gCurr_alpha_beta, &gTrig, &gCurr_d_q);
            
//            gCurr_d_q_filter.i_d = gCurr_d_q_filter.i_d * FILTER_COEFF_D_Q + gCurr_d_q.i_d * (1-FILTER_COEFF_D_Q);
//            gCurr_d_q_filter.i_q = gCurr_d_q_filter.i_q * FILTER_COEFF_D_Q + gCurr_d_q.i_q * (1-FILTER_COEFF_D_Q);
            
            /*IQ pid*/
            Pid(gFoc.IqRef, gCurr_d_q.i_q, &gPID_IQ);
            
            /*ID pid*/
            Pid(gFoc.IdRef, gCurr_d_q.i_d, &gPID_ID);
            
            gVolt_d_q.u_q = gPID_IQ.Out;
            gVolt_d_q.u_d = gPID_ID.Out;

            /*revpark*/
            RevPark(&gVolt_d_q, &gTrig, &gVolt_alpha_beta);
            /*svpwm*/
            Svpwm(&gVolt_alpha_beta, &gSvpwm);

            /*set pwm*/
            mc_PwmLoad(gSvpwm.PWM_A, gSvpwm.PWM_B, gSvpwm.PWM_C);
            break;
                
        case MC_STATE_IDLE:
            /*set out 0*/
            mc_PwmLoad(gSvpwm.PWMPeriod/2, gSvpwm.PWMPeriod/2, gSvpwm.PWMPeriod/2);
            break;
            
        default:break;
    }
    
    /*read electrical angle*/
    mc_EncoderStartRead();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(gFoc.Angle * 10));//
}

void mc_MotorDirCoeffGet(HallStructure *hall)
{
    int8_t i,j;
    
    for(i=0,j=0; i<5; i++)
    {
        if(hall->StudyDirCode[i] < hall->StudyDirCode[i+1])
            j++;
        else
            j--;
    }
    
    if(j > 0)
        hall->MotoRunDirCoeff = 1.0f;
    else
        hall->MotoRunDirCoeff = -1.0f;
}

void mc_MotorHallStudy(mcParam_FOC * foc, HallStructure *hall, mcParam_Encoder  *encoder)
{
	static uint32_t Cnt = 0;
    if(hall->StudyOk)
        return;
    
	if(0 == hall->StudyStart)
	{
        hall->StudyStart = 1;
		foc->Angle = 0;
		foc->IqRef = 0;
    	foc->IdRef = 1.5;
	}
	
	Cnt++;
	if(Cnt >= 1000)
	{
        Cnt = 0;
		mc_HallSetAngleCode(hall, foc->Angle);
        mc_EncoderDirCoeffGet();
		foc->Angle += 60;
		if(foc->Angle >= 360)
		{
			hall->StudyOk = 1;
			hall->StudyStart  = 0;
			foc->IqRef = 0;
    		foc->IdRef = 0;
            mc_MotorDirCoeffGet(hall);
		}
	}
}

void mc_UserSet(uint8_t cmd, uint8_t *Dat)
{
    Cmd_Convert_Structure   CmdConvert;   

    CmdConvert.Byte[0] = Dat[3];
    CmdConvert.Byte[1] = Dat[2];
    CmdConvert.Byte[2] = Dat[1];
    CmdConvert.Byte[3] = Dat[0];
    
    switch(cmd)
    {
        /*ON OFF*/
        case MC_USER_SET_ON_OFF:
            gFoc.RunMode   = MC_USER_SET_ON_OFF;
            gFoc.MotoStart = CmdConvert.uintDat;
            break;
        
        /*switch idle*/
        case MC_USER_SET_IDLE:
            gFoc.RunMode   = MC_USER_SET_IDLE;
            gFoc.MotoStart = 0;
            gFoc.IqRef     = 0;
            gFoc.IdRef     = 0;
            gFoc.RpmRef    = 0;
            gFoc.MechaAngleRef = 0;
        
            break;
        
        /*switch open run*/
        case MC_USER_SET_OPEN_RUN:
            if(MC_STATE_IDLE == gFoc.RunStep)
            {
                gFoc.RunMode   = MC_USER_SET_OPEN_RUN;
                gFoc.MotoStart = 1;
                gFoc.IqRef     = CmdConvert.floatDat;
            }
            break;
        
        /*speed loop*/   
        case MC_USER_SET_SPEED:
            if((MC_STATE_IDLE == gFoc.RunStep)||(MC_STATE_RUN == gFoc.RunStep))
            {
                gFoc.RunMode   = MC_USER_SET_SPEED;
                gFoc.MotoStart = 1;
                gFoc.RpmRef    = CmdConvert.floatDat;
            }
            break;
        
        /*postion loop*/
        case MC_USER_SET_POSITION:
            if((MC_STATE_IDLE == gFoc.RunStep)||(MC_STATE_RUN == gFoc.RunStep))
            {
                gFoc.RunMode       = MC_USER_SET_POSITION;
                gFoc.MotoStart     = 1;
                gFoc.MechaAngleRef = CmdConvert.intDat;
            }
            break;
        
        /*hall study*/
        case MC_USER_SET_HALL_STUDY:
            if(MC_STATE_IDLE == gFoc.RunStep)
            {
                gHall.StudyOk  = 0;
                gFoc.RunStep   = MC_STATE_HALL_STUDY;
            }
            break;
        
        /*set orientation*/
        case MC_USER_SET_ORIENTATION:
            if((MC_STATE_IDLE == gFoc.RunStep)&&(0 == gHall.Omega))
            {
                mc_EncoderSetOrigin();
            }
            break;
        
        /*set speed limit*/
        case MC_USER_SET_SPEED_LIMIT:
            gPID_Position.OutMax = CmdConvert.floatDat;
            gPID_Position.OutMin = -CmdConvert.floatDat;
            
            break;
         
        /*set torque limit*/
        case MC_USER_SET_TORQUE_LIMIT:
            gPID_Speed.OutMax = CmdConvert.floatDat * gFoc.TorqueCoeff;
            gPID_Speed.OutMin = -CmdConvert.floatDat * gFoc.TorqueCoeff;
            break;
        
        case MC_USER_SET_TORQUE_COEFF:
            gFoc.TorqueCoeff = CmdConvert.floatDat / gFoc.IqRef;
            break;
            
        
        case MC_USER_SET_CURR_P:
            gPID_IQ.Kp = CmdConvert.floatDat;
            gPID_ID.Kp = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_CURR_I:
            gPID_IQ.Ki = CmdConvert.floatDat;
            gPID_ID.Ki = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_SPEED_P:
            gPID_Speed.Kp = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_SPEED_I:
            gPID_Speed.Ki = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_POS_P:
            gPID_Position.Kp = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_POS_I:
            gPID_Position.Ki = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_POS_D:
            gPID_Position.Kd = CmdConvert.floatDat;
            break;
        
        case MC_USER_SET_DEFAULT:
            mc_ParamDefault();
            break;
        
        case MC_USER_READ_POSITION:
            sprintf((char *)CmdTxBuf,"Position:%.3f\r\n",gFoc.MechaAngleMeas);
            HAL_UART_Transmit_IT(&huart1, CmdTxBuf, strlen((char *)CmdTxBuf));
            break;
        
        default:break;
    }
}

void mc_HallTimerOut(TIM_HandleTypeDef *htim)
{
    mc_HallTimeOut(&gHall);
}

/**
  * @brief speed closed loop ,position closed loop
  * @note  speed control 2K,position control period 1k
  */
float gMechaAngleFilter = 0.0f;
void mc_ControlLoop(TIM_HandleTypeDef *htim)
{
	static uint32_t  sPositionLoopPeriod = 0;
	float AngleCurr, AngleLast, AngleDelat;
	static uint32_t DebugCnt = 0;

//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,1);
    switch(gFoc.RunStep)
    {
        case MC_STATE_ORIENTATION:
            break;
        
        case MC_STATE_HALL_STUDY:
            mc_MotorHallStudy(&gFoc, &gHall, &gEncoder);
            break;
        
        case MC_STATE_RUN:

            if(MC_USER_SET_SPEED == gFoc.RunMode)
            {   
//                mc_HallGetParam(&gFoc.RpmMeas, HALL_RPM);
                mc_EncoderGetParam(&gFoc.RpmMeas, ENCODER_PARAM_MECHA_OMEGA);
                Pid(gFoc.RpmRef, gFoc.RpmMeas, &gPID_Speed);

                gFoc.IqRef = gPID_Speed.Out;
                gFoc.IdRef = 0;                   
            }
            else if(MC_USER_SET_POSITION == gFoc.RunMode)
            {
                sPositionLoopPeriod++;
                if(sPositionLoopPeriod >= 2)
                {
                    sPositionLoopPeriod = 0;
                    mc_EncoderGetParam(&gFoc.MechaAngleMeas, ENCODER_PARAM_MECHA_ANGLE);
                    gMechaAngleFilter = gMechaAngleFilter * 0.9 + gFoc.MechaAngleMeas * 0.1;
                    Pid(gFoc.MechaAngleRef, gMechaAngleFilter, &gPID_Position);
                }
                
                gFoc.RpmRef = gPID_Position.Out;
                mc_EncoderGetParam(&gFoc.RpmMeas, ENCODER_PARAM_MECHA_OMEGA);
                Pid(gFoc.RpmRef, gFoc.RpmMeas, &gPID_Speed);
                
                gFoc.IqRef = gPID_Speed.Out;
                gFoc.IdRef = 0;
            }
            else if(MC_USER_SET_TORQUE == gFoc.RunMode)
            {
                gFoc.IdRef = 0;
            }
            else
            {
                gFoc.IqRef = 0;
                gFoc.IdRef = 0;
            }
            break;
            
        case MC_STATE_USER:         
            gFoc.Angle += 0.05;
            if(gFoc.Angle >= 360)
            {
                gFoc.Angle = 0;
            }
        
//            gFoc.Angle = 0;
//            DebugCnt++;
//            if(DebugCnt < 3000)
//                gFoc.IdRef = 3;
//            else if(DebugCnt < 6000)
//                gFoc.IdRef = 1;
//            else
//                DebugCnt = 0;
            
            gFoc.IqRef = 0;
            gFoc.IdRef = 1;
            break;

        default:
            sPositionLoopPeriod = 0;
            gFoc.IqRef = 0;
            gFoc.IdRef = 0;
            gPID_Position.Sum = 0;
            gPID_Speed.Sum   = 0; 
            break;
    }
    
	mc_EncoderPeriodCal(&gEncoder, MC_SPEED_LOOP_PERIOD);
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0);
}


extern uint8_t gUartRxData;
void mc_FocStateMachine(void)
{
	switch(gFoc.RunStep)
	{
		case MC_STATE_INIT:
			mc_ParamInit();
			mc_EncoderInit();
            CmdInit();
            HAL_UART_Transmit_IT(&huart1, gTitle, sizeof(gTitle));
            HAL_UART_Receive_IT(&huart1, &gUartRxData, 1);
			gFoc.RunStep = MC_STATE_BIAS;
            mc_OnOffCtr(1);
			break;
		
		case MC_STATE_BIAS:
            mc_OnOffCtr(1);
			if(gFoc.OffsetGetOk)
			{
				gFoc.RunStep = MC_STATE_HALL_STUDY;//MC_STATE_USER;//MC_STATE_IDLE;
			}
			break;
            
        case MC_STATE_HALL_STUDY:
            mc_OnOffCtr(1);
            if(gHall.StudyOk)
			{
				gFoc.RunStep = MC_STATE_ORIENTATION;
                gFoc.MotoStart = 0;
				gFoc.MotoStatus = 0;
                mc_OnOffCtr(0);
			}
            break;
        
		case MC_STATE_ORIENTATION:
			if(0 == gEncoder.OrientationOk)
			{
                mc_EncoderSetOrigin();
                gEncoder.OrientationOk = 1;
			}
			else
			{
				gFoc.RunStep = MC_STATE_IDLE;
			}
			break;
				
		case MC_STATE_IDLE:

            mc_EncoderGetParam(&gFoc.MechaAngleMeas, ENCODER_PARAM_MECHA_ANGLE);
            if(gFoc.MotoStart)
            {
                if(0 == gFoc.MotoStatus)
                {
                    gFoc.RunStep = MC_STATE_USER;
                    gFoc.RunStep = MC_STATE_RUN;
                    mc_HallCapture(&gHall, &htim17);
                    gFoc.MotoStatus = 1;
                }
            }
            gFoc.IqRef = 0;
            gFoc.IdRef = 0;
            mc_OnOffCtr(1);
			break;
				
		case MC_STATE_RUN:
			if(0 == gFoc.MotoStart)
			{
                if(0 == gFoc.RunMode)
                {
                    gFoc.RunStep = MC_STATE_STOP;
                    
                }
                else if(MC_USER_SET_IDLE == gFoc.RunMode)
                {
                    gFoc.RunStep    = MC_STATE_IDLE;
                }
                gFoc.MotoStatus = 0;
			}
			break;
			
		case MC_STATE_STOP:
            mc_OnOffCtr(0);
            if(0 != gFoc.RunMode)
            {
                gFoc.RunStep = MC_STATE_IDLE;
                mc_OnOffCtr(1);
            }
			break;
			
		case MC_STATE_FAULT:
		
			break;
			
		case MC_STATE_USER:
				gFoc.MotoStart = 1;
				gFoc.MotoStatus = 1;
			break;
			
		default:
			gFoc.RunStep = MC_STATE_INIT;
			break;
	}	
}



