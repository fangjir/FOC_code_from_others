
#ifndef _MC_USER_PARAM_
#define _MC_USER_PARAM_

/* PI controllers tuning values - */

/********* D Control Loop Coefficients ****************************************/
#define     D_CURRCNTR_PTERM           (float)(0.033)
#define     D_CURRCNTR_ITERM           (float)(0.000335)
#define     D_CURRCNTR_CTERM           (float)(0.5)
#define     D_CURRCNTR_OUTMAX          (float)(0.9)

/******** Q Control Loop Coefficients ****************************************/
#define     Q_CURRCNTR_PTERM           (float)(0.033)
#define     Q_CURRCNTR_ITERM           (float)(0.000335)
#define     Q_CURRCNTR_CTERM           (float)(0.5)
#define     Q_CURRCNTR_OUTMAX          (float)(0.9)

/******* Velocity Control Loop Coefficients **********************************/
#define     SPEEDCNTR_PTERM            (float)(0.0445)
#define     SPEEDCNTR_ITERM            (float)(0.000135)
#define     SPEEDCNTR_DTERM            (float)(1.27)
#define     SPEEDCNTR_CTERM            (float)(0.0)
#define     SPEEDCNTR_OUTMAX           (float)(4.0)

/*** Position Control Loop Coefficients *****/
#define     POSCNTR_PTERM               (float)(10.0)
#define     POSCNTR_ITERM               (float)(0.0)
#define     POSCNTR_DTERM               (float)(40.0)
#define     POSCNTR_CTERM               (float)(0.0)
#define     POSCNTR_OUTMAX              (500)

#endif
