#include "stdio.h"
#include "main.h"
#include "cmd.h"
#include "MotorControl.h"

uint8_t gUartRxData;
Cmd_Member_Structure    gCmd;

uint8_t CmdRxBuf[CMD_DATA_LENGHT];
uint8_t CmdTxBuf[CMD_DATA_LENGHT];
extern UART_HandleTypeDef huart1;

uint8_t gCmdTest[] = "Cmd Recv Ok !";
void CmdInit(void)
{
    gCmd.RxDat = CmdRxBuf;
    gCmd.TxDat = CmdTxBuf;
    gCmd.RxCnt = 0;
    gCmd.RxPos = 0;
}


void CmdRecv(uint8_t Dat)
{
    uint8_t checksum = 0;
	uint8_t i;
    
    if(1 == gCmd.RxOK)
    {
        gCmd.RxPos = 0;
        return;
    }
    gCmd.RxDat[gCmd.RxPos++] = Dat;
    if(gCmd.RxPos >= CMD_FRAME_LENGTH)
    {
        if((0xFF == gCmd.RxDat[0])&&(0xAA == gCmd.RxDat[1]))
		{
			for(i=0; i<CMD_FRAME_LENGTH-1; i++)
			{
				checksum += gCmd.RxDat[i];
			}
			if(checksum == gCmd.RxDat[CMD_FRAME_LENGTH-1])
			{
                gCmd.RxOK = 1;
				gCmd.RxPos = 0;
			}
            else
            {
                gCmd.RxPos = 0;
            }
		}
		else
		{
			for(i=0;i<CMD_FRAME_LENGTH-1;i++)
			{
			   gCmd.RxDat[i] =  gCmd.RxDat[i+1];
			}
			gCmd.RxPos = CMD_FRAME_LENGTH-1;
		}
    }
}

void CmdSend(UART_HandleTypeDef *huart)
{
    
}    

void CmdLoop(void)
{
    if(gCmd.RxOK)
    {
        mc_UserSet(gCmd.RxDat[2], &gCmd.RxDat[3]);
//        HAL_UART_Transmit_IT(&huart1, gCmdTest, sizeof(gCmdTest));
//        sprintf((char *)CmdTxBuf,"Position:%.3f\r\n",gFoc.MechaAngleMeas);
        gCmd.RxOK = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1)
    {
        HAL_UART_Receive_IT(&huart1, &gUartRxData, 1);
        CmdRecv(gUartRxData);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1)
    {
        huart->Instance->ISR;
    }
}
