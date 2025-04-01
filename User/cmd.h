
#ifndef _CMD_H_
#define _CMD_H_

#define CMD_DATA_LENGHT     100
#define CMD_FRAME_LENGTH    8
typedef struct
{
    uint8_t RxCnt;
    uint8_t RxOK;
    uint8_t RxPos;
    
    uint8_t TxPos;
    uint8_t TxLength;
    uint8_t TxEn;
    
    uint8_t *RxDat;
    uint8_t *TxDat;
}Cmd_Member_Structure;

extern uint8_t gUartRxData;
extern Cmd_Member_Structure gCmd;
extern uint8_t CmdRxBuf[CMD_DATA_LENGHT];
extern uint8_t CmdTxBuf[CMD_DATA_LENGHT];
extern void CmdInit(void);
extern void CmdLoop(void);

#endif
