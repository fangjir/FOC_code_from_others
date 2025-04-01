#ifndef _HALL_H_
#define _HALL_H_

#define  HALL_ACC_ANGLE   1
#define  HALL_SYNC_ANGLE  2
#define  HALL_OMEGA       3
#define  HALL_RPM         4
#define  HALL_DIR         5
#define  HALL_VALUE       6
typedef struct
{
    int32_t StudyOk;
    int32_t StudyStart;
	int32_t Dir;           /*0 : stop, 1:CW,  -1:CCW*/
	int32_t Value;
	int32_t ValueLast;
	int32_t TimeOutCnt;
    int32_t TimeTemp1;
    int32_t TimeTemp2;
	int32_t TimeDelat;
    int32_t TimeDeltaFilterCnt;
    int32_t TimeDeltaFilterDat[6];
    int32_t TimeDeltaNoFilter;
    
    float   MotoRunDirCoeff;
	float   AngleCode[8];  /*AngleCode[0]:have study*/
    float   StudyDirCode[8];
    uint32_t   StudyDirCnt;
	float   Omega;
	float   OmegaFilterDat[6];
	int32_t   OmegaFilterCnt;
	float   Rpm;
    float   AngleSync;
	float   AngleAcc;
    
	float   AngleDpp;
    float   AngleDppStart;
    float   AngleDppSumPerSect;	
    float   TestCnt;
}HallStructure;

extern HallStructure gHall;

extern void mc_HallInit(HallStructure *hall);
extern void mc_HallAngleCal(HallStructure *hall);
extern void mc_HallGetParam(void *Target, uint8_t ParamType);
extern void mc_HallSetAngleCode(HallStructure *hall, float Angle);
extern void mc_HallTimeOut(HallStructure *hall);
extern void mc_HallCapture(HallStructure *hall, TIM_HandleTypeDef *htimer);
#endif
