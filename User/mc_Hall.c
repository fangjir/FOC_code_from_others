#include "main.h"
#include "mc_Lib.h"
#include "mc_Hall.h"

#define GPIO_READ_HALL   GPIOC->IDR
HallStructure gHall;

void mc_HallInit(HallStructure *hall)
{
	hall->Dir   = 0;
	hall->Value = 0;
	hall->ValueLast = 0;
	hall->AngleCode[0] = 0; 
	hall->AngleCode[1] = 0; 
	hall->AngleCode[2] = 0; 
	hall->AngleCode[3] = 0; 
	hall->AngleCode[4] = 0; 
	hall->AngleCode[5] = 0; 
	hall->AngleCode[6] = 0; 
	hall->AngleCode[7] = 0; 
	hall->Omega        = 0;
	hall->AngleSync       = 0;
}

int32_t mc_HallDeltaTimeAverage(int32_t *Dat, int32_t Num)
{
    int32_t i,j,Tmp;
//    for(i=0; i<Num-1; i++)
//    {
//        for(j=0; j<Num-i; j++)
//        {
//            if(Dat[j] < Dat[j+1])
//            {
//                Tmp = Dat[j+1];
//                Dat[j+1] = Dat[j];
//                Dat[j] = Tmp;
//            }
//        }
//    }
    
    return (Dat[2] + Dat[3])/2;
}

float mc_HallDeltaOmegaAverage(float *Dat, int32_t Num)
{
    int32_t i,j;
    float Tmp;
//    for(i=0; i<Num-1; i++)
//    {
//        for(j=0; j<Num-i; j++)
//        {
//            if(Dat[j] < Dat[j+1])
//            {
//                Tmp = Dat[j+1];
//                Dat[j+1] = Dat[j];
//                Dat[j] = Tmp;
//            }
//        }
//    }
    
    return (Dat[2] + Dat[3])/2;
}

/**
  * @brief sync angle @ edge
  */
extern TIM_HandleTypeDef htim16;

void mc_HallCapture(HallStructure *hall, TIM_HandleTypeDef *htimer)
{
	int32_t HallValue = 0;
    float OmegaTmp = 0, Dir = 0;
    static float Angle1 = 0, Angle2 = 0, DirFilter = 0, DirCnt = 0;
    
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);
    
    if(0 == hall->StudyOk)
	{
        return;
    }

    while(1)
    {
        HallValue = ((GPIO_READ_HALL) >> 13) & 0x07;
        if((HallValue > 0)&&(HallValue < 7))
        {
            if(hall->Value == HallValue)
            {
                return;
            }
            hall->Value = HallValue;
            break;
        }
    }
      
    /*ͬ���ǶȺ󣬰�ÿ�����ۼӺ�����*/
    hall->AngleDppSumPerSect = 0;

    /*get angle and angle delta*/
	hall->AngleSync = hall->AngleCode[hall->Value] - hall->Dir * 30 ;
    
    
    if(hall->AngleSync >= 360)
    {
        hall->AngleSync -= 360;
    }
    else if(hall->AngleSync < 0)
    {
        hall->AngleSync += 360;
    }
        
    /*time delta cal*/
    hall->TimeTemp1  = htimer->Instance->CNT;
    hall->TimeDeltaNoFilter = hall->TimeOutCnt * 65536 + hall->TimeTemp1 - hall->TimeTemp2;
    hall->TimeDeltaFilterDat[hall->TimeDeltaFilterCnt++]  = hall->TimeDeltaNoFilter;
    hall->TimeTemp2  = hall->TimeTemp1;
    hall->TimeOutCnt = 0;
    
    if(hall->TimeDeltaFilterCnt >= 6)
    {
        hall->TimeDeltaFilterCnt = 0;
    }
    
    hall->TimeDelat = mc_HallDeltaTimeAverage(hall->TimeDeltaFilterDat, 6);
	OmegaTmp = ((float)60.0) * ((float)1000000.0) / hall->TimeDelat; //1us per cnt;
    
    hall->OmegaFilterDat[hall->OmegaFilterCnt++] = OmegaTmp;
	if(hall->OmegaFilterCnt >= 6)
	{
		hall->OmegaFilterCnt = 0;
	}
	OmegaTmp = mc_HallDeltaOmegaAverage(hall->OmegaFilterDat, 6);
        
    /*get direction*/
    Angle1 = hall->AngleSync;
    if((Angle1 < 90)&&(Angle2 > 270))
    {
        Dir = 1;
    }
    else if((Angle1 > 270)&&(Angle2 < 90))
    {
        Dir = -1;
    }
    else
    {
        if(Angle1 > Angle2)
            Dir = 1;
        else if(Angle1 < Angle2)
            Dir = -1;
        else
            Dir = 0;
    }
    Angle2 = Angle1;
    /*end*/
    
    if(DirFilter != Dir)
    {
        DirCnt++;
        if(DirCnt > 6)
        {
            DirFilter = Dir;
        }
    }
    else
    {
        DirCnt = 0;
    }
    
    if(OmegaTmp > 720)
    {
        hall->Dir = DirFilter;
        hall->AngleDppStart = 1;
    }
    else
    {
        hall->Dir =  0;
        hall->AngleDppStart = 0;
    }
    
    hall->Omega = hall->Dir * OmegaTmp;
    hall->Rpm   = hall->Omega / ((float)60.0) / ((float)POLE);
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
}

/**
  * @brief Timer over
  * @mote ��ʱ��ʱ�����һ�νǶȣ���֤��ת�ٵ�ʱ�����ܱ��ִ�Ť�أ�ͬʱ��ֹ�Ƕ��ۼ�
  */

void mc_HallTimeOut(HallStructure *hall)
{
    int32_t HallValue;
	hall->TimeOutCnt++;
	if(hall->TimeOutCnt >= 2)
	{
        hall->Dir       = 0;
        hall->AngleDppStart = 0;
        hall->AngleDppSumPerSect = 0;

        while(1)
        {
            HallValue = ((GPIO_READ_HALL) >> 13) & 0x07;
            if((HallValue > 0)&&(HallValue < 7))
            {
                hall->Value = HallValue;
                break;
            }
        }
        hall->AngleSync = hall->AngleCode[hall->Value];
        hall->Omega = 0;
        hall->Rpm   = 0;
    }
}

/**
  * @brief read hall angle
  * @note  run in foc
  */
void mc_HallAngleCal(HallStructure *hall)
{
	hall->AngleDpp = hall->Omega / ((float)PWM_FREQ);
    if(hall->AngleDppStart)
    {
        if((hall->AngleDppSumPerSect < 60)&&(hall->AngleDppSumPerSect > -60))
        {
            hall->AngleAcc            = hall->AngleSync + hall->AngleDppSumPerSect;
            hall->AngleDppSumPerSect  = hall->AngleDppSumPerSect + hall->AngleDpp;
        }
    }
    else
    {
        hall->AngleAcc = hall->AngleSync;
    }
    
    if(hall->AngleAcc >= 360)
        hall->AngleAcc -= 360;
    else if(hall->AngleAcc < 0)
        hall->AngleAcc += 360;
}

/**
  * @brief extern get hall param
  */
void mc_HallGetParam(void *Target, uint8_t ParamType)
{
	switch(ParamType)
	{
		case HALL_ACC_ANGLE:
			*((float *)Target) = gHall.AngleAcc;
			break;
        
        case HALL_SYNC_ANGLE:
			*((float *)Target) = gHall.AngleSync;
			break;
			
		case HALL_OMEGA:
			*((float *)Target) = gHall.Omega;
			break;

		case HALL_RPM:
			*((float *)Target) = gHall.Rpm;
			break;
			
		case HALL_DIR:
			*((int32_t *)Target) = gHall.Dir;
			break;		
		
		case HALL_VALUE:
			*((int32_t *)Target) = gHall.Value;
			break;
		
		default:
			break;
	}
}

void mc_HallSetAngleCode(HallStructure *hall, float Angle)
{
	hall->Value = ((GPIO_READ_HALL) >> 13) & 0x07;
    if((hall->Value < 1)||(hall->Value >= 7))
        return;
    hall->AngleCode[hall->Value] = Angle;
}

void mc_HallAngleInit(HallStructure *hall)
{
	hall->Omega 	= 0;
	hall->Rpm		= 0;
	hall->Value 	= ((GPIO_READ_HALL) >> 13) & 0x07;
    hall->AngleSync     = hall->AngleCode[hall->Value];
}


extern TIM_HandleTypeDef htim17;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if((GPIO_Pin == GPIO_PIN_13) || (GPIO_Pin == GPIO_PIN_14) || (GPIO_Pin == GPIO_PIN_15))
    {
        mc_HallCapture(&gHall, &htim17);
    }
}

