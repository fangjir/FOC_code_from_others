
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "mc_Lib.h"

#define MC_STATE_INIT          1
#define MC_STATE_BIAS          2
#define MC_STATE_ORIENTATION   3   
#define MC_STATE_HALL_STUDY    4 
#define MC_STATE_IDLE          5
#define MC_STATE_RUN           6
#define MC_STATE_STOP          7
#define MC_STATE_FAULT         8
#define MC_STATE_USER          9

#define MC_USER_SET_ON_OFF         1
#define MC_USER_SET_IDLE           2
#define MC_USER_SET_OPEN_RUN       3
#define MC_USER_SET_SPEED          4
#define MC_USER_SET_POSITION       5
#define MC_USER_SET_TORQUE         6
#define MC_USER_SET_TORQUE_COEFF   7
#define MC_USER_SET_HALL_STUDY     8
#define MC_USER_SET_ORIENTATION    9
#define MC_USER_SET_SPEED_LIMIT    10 
#define MC_USER_SET_TORQUE_LIMIT   11

#define MC_USER_SET_CURR_P         50
#define MC_USER_SET_CURR_I         51
#define MC_USER_SET_SPEED_P        52
#define MC_USER_SET_SPEED_I        53
#define MC_USER_SET_POS_P          54
#define MC_USER_SET_POS_I          55
#define MC_USER_SET_POS_D          56
#define MC_USER_SET_DEFAULT        57

#define MC_USER_READ_POSITION      100



#define MC_STATUS                  255


#define mc_PwmOutputEnable()       R_PORT_SetGpioOutput(Port0, 6, 0)
#define mc_PwmOutputDisable()      R_PORT_SetGpioOutput(Port0, 6, 1)

#define mc_DebugGpio1H()            R_PORT_SetGpioOutput(Port9, 6, 1)
#define mc_DebugGpio1L()            R_PORT_SetGpioOutput(Port9, 6, 0)

#define mc_DebugGpio2H()            R_PORT_SetGpioOutput(Port10, 7, 1)
#define mc_DebugGpio2L()            R_PORT_SetGpioOutput(Port10, 7, 0)


#define MC_BIAS_IDEAL         2047   /* 1.65 / 3.3 * 4095*/
#define MC_BIAS_ERR_MAX       372    /*0.3V / 3.3V * 4095*/  
#define MC_SAMPLE_R           0.05
#define MC_SAMPLE_A           7.5
#define MC_CURR_CONV_COEFF    ((float)(3.3 / 4095.0 / MC_SAMPLE_R / MC_SAMPLE_A))
#define MC_CURR_CONV_COEFF2   ((float)(3.3 / 4095.0 * 11000.0 / 2495.0))


#define MC_SPEED_LOOP_FREQ      ((float)2000.0)
#define MC_SPEED_LOOP_PERIOD    ((float)(1.0 / MC_SPEED_LOOP_FREQ))
#define MC_POSITION_LOOP_FFREQ  ((float)500.0)
#define MC_POSITION_LOOP_PERIOD ((float)(1.0 / MC_POSITION_LOOP_FFREQ))

#define MC_CURR_FILTER_COEFF     ((float)0.99)
typedef struct 
{
	uint8_t MotoStart;
	uint8_t MotoStatus;

	uint8_t RunStep;
	uint8_t RunMode;
	uint8_t OffsetGetOk;
    uint8_t OrientationOk;
    
    uint32_t AdcNtc;
    
	float   Angle;     /*电角度*/
	float   StudyAngle;
	
	float   MechaAngleRef;    /*机械角度*/
	float   MechaAngleMeas;  //机械角度测量值
	float   ElecOmegaRef;
	float   ElecOmegaMeas;
	float   MechaOmegaRef;
	float   MechaOmegaMeas;
    float   RpmRef;
	float   RpmMeas;

	float   IqRef;
	float   IqMeas;
	float   IdRef;
	float   IdMeas;
    
    float   TorqueCoeff;

	uint32_t OrientaCnt;
    uint32_t TestCnt;
}mcParam_FOC;

typedef union
{
    uint8_t  Byte[4];
    int32_t  intDat;
    uint32_t uintDat;
    float    floatDat;
}Cmd_Convert_Structure;


extern mcParam_Curr_abc          gCurr_abc;
extern mcParam_Curr_alpha_beta   gCurr_alpha_beta;
extern mcParam_Curr_d_q          gCurr_d_q;
extern mcParam_Volt_alpha_beta   gVolt_alpha_beta;
extern mcParam_Volt_d_q          gVolt_d_q;
extern mcParam_Trigonometric     gTrig;
extern mcParam_SVPWM             gSvpwm;

extern mcParam_PID               gPID_IQ;
extern mcParam_PID               gPID_ID;
extern mcParam_PID               gPID_Speed;
extern mcParam_PID               gPID_Position;

extern mcParam_FOC               gFoc;

extern void mc_FocInit(void);
extern void mc_MotoStop(void);
extern void mc_MotoStart(void);
extern void mc_CoreAlgorithm(ADC_HandleTypeDef *hadc);
extern void mc_ControlLoop(TIM_HandleTypeDef *htim);
extern void mc_UserSet(uint8_t cmd, uint8_t *Dat);
extern void mc_FocStateMachine(void);
extern uint8_t mc_CmdDeal(uint8_t Cmd, uint8_t * Dat);
extern void mc_HallTimerOut(TIM_HandleTypeDef *htim);
extern void mc_HallMaskDisable(TIM_HandleTypeDef *htim);
#endif
