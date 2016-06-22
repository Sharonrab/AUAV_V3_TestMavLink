#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER.h"
#include "AUAV3_WITH_SLUGS_SENSOR_AND_CONTROLLER_private.h"

/* I2C - Interrupt: <S12>/BUS I2C Initialize BMP180 Read T°, Convert P @ 100Hz */
/* Implement I2C 2 Interrupts */
void __attribute__((__interrupt__,__auto_psv__)) _MI2C2Interrupt(void) /* MI2C2 ISR */
{
  boolean_T Continue;
  _MI2C2IF = 0;

  /* Declaration of Variables */
  do {
    Continue = 0;                      /* By default, execute one sequence only */
    switch (MCHP_I2C2_State)
    {
     case 0:                           /* Idle */
      /* Check if the next sequence is already in the queue. If so, start it */
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      if (Continue) {
        MCHP_I2C2_State = MCHP_I2C2_Queue.buffer[MCHP_I2C2_Queue.tail];
        if (MCHP_I2C2_Queue.tail >= 5) /* There are 5 I2C2 blocks, max idx for queue is 5 */
          MCHP_I2C2_Queue.tail = 0;
        else
          MCHP_I2C2_Queue.tail++;
      }
      break;

     case 1:                           /* Problem: try Reset of I2C BUS : Repeated Start sequence and Stop */
      TRISAbits.TRISA3 = 1;            /* Set I2C Port as Port Input (better for Fj chips)*/
      TRISAbits.TRISA2 = 1;
      I2C2CONbits.I2CEN = 1;           /* Re-Enable I2C*/
      I2C2STAT &= 65526;               /* Reset I2C State */
      I2C2CONbits.RSEN = 1;            /* I2C: Restart */
      MCHP_I2C2_State++;
      break;

     case 2:                           /* Problem: try Reset of I2C BUS : Stop sequence*/
      I2C2STAT &= 65526;               /* Reset I2C Receive Overflow and I2C Collision during Write or Master operation  */
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 3:
      I2C2STAT &= 65526;               /* Reset I2C Receive Overflow and I2C Collision during Write or Master operation  */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* Any pending sequence ? */
      break;

     case 4:                           /* Start a new I2C Sequence */
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 5:
      I2C2TRN = 0xEE;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 6:
      I2C2TRN = 0xF6;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 7:
      I2C2CONbits.RSEN = 1;            /* I2C: REPEATED START */
      MCHP_I2C2_State++;
      break;

     case 8:
      I2C2TRN = 0xEF;                  /* I2C : Address Repeat */
      MCHP_I2C2_State++;
      break;

     case 9:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 10:
      I2C22_Buff8[0] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 11:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 12:
      I2C22_Buff8[1] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 1;           /* set to NACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 13:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 14:
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 15:
      I2C2TRN = 0xEE;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 16:
      I2C2TRN = 0xF4;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 17:
      I2C2TRN = 0x34;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 18:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 19:
      MCHP_I2C22_Request = 0;          /* This I2C sequence could be Re-Enabled */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      break;

     case 20:                          /* Start a new I2C Sequence */
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 21:
      I2C2TRN = 0xEE;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 22:
      I2C2TRN = 0xF6;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 23:
      I2C2CONbits.RSEN = 1;            /* I2C: REPEATED START */
      MCHP_I2C2_State++;
      break;

     case 24:
      I2C2TRN = 0xEF;                  /* I2C : Address Repeat */
      MCHP_I2C2_State++;
      break;

     case 25:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 26:
      I2C23_Buff8[0] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 27:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 28:
      I2C23_Buff8[1] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 1;           /* set to NACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 29:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 30:
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 31:
      I2C2TRN = 0xEE;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 32:
      I2C2TRN = 0xF4;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 33:
      I2C2TRN = 0x2E;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 34:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 35:
      MCHP_I2C23_Request = 0;          /* This I2C sequence could be Re-Enabled */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      break;

     case 36:                          /* Start a new I2C Sequence */
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 37:
      I2C2TRN = 0xEE;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 38:
      I2C2TRN = 0xAA;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 39:
      I2C2CONbits.RSEN = 1;            /* I2C: REPEATED START */
      MCHP_I2C2_State++;
      break;

     case 40:
      I2C2TRN = 0xEF;                  /* I2C : Address Repeat */
      MCHP_I2C2_State++;
      break;

     case 41:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 42:
      I2C21_Buff8[0] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 43:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 44:
      I2C21_Buff8[1] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 45:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 46:
      I2C21_Buff8[2] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 47:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 48:
      I2C21_Buff8[3] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 49:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 50:
      I2C21_Buff8[4] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 51:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 52:
      I2C21_Buff8[5] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 53:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 54:
      I2C21_Buff8[6] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 55:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 56:
      I2C21_Buff8[7] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 57:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 58:
      I2C21_Buff8[8] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 59:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 60:
      I2C21_Buff8[9] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 61:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 62:
      I2C21_Buff8[10] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 63:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 64:
      I2C21_Buff8[11] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 65:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 66:
      I2C21_Buff8[12] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 67:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 68:
      I2C21_Buff8[13] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 69:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 70:
      I2C21_Buff8[14] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 71:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 72:
      I2C21_Buff8[15] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 73:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 74:
      I2C21_Buff8[16] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 75:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 76:
      I2C21_Buff8[17] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 77:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 78:
      I2C21_Buff8[18] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 79:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 80:
      I2C21_Buff8[19] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 81:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 82:
      I2C21_Buff8[20] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 83:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 84:
      I2C21_Buff8[21] = I2C2RCV;       /* I2C : Read data */
      I2C2CONbits.ACKDT = 1;           /* set to NACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 85:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 86:
      MCHP_I2C21_Request = 0;          /* This I2C sequence could be Re-Enabled */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      break;

     case 87:                          /* Start a new I2C Sequence */
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 88:
      I2C2TRN = 0x3C;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 89:
      I2C2TRN = 0x03;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 90:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 91:
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 92:
      I2C2TRN = 0x3D;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 93:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 94:
      I2C24_Buff8[0] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 95:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 96:
      I2C24_Buff8[1] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 97:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 98:
      I2C24_Buff8[2] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 99:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 100:
      I2C24_Buff8[3] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 101:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 102:
      I2C24_Buff8[4] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 0;           /* set to ACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 103:
      I2C2CONbits.RCEN = 1;            /* I2C: Receive  */
      MCHP_I2C2_State++;
      break;

     case 104:
      I2C24_Buff8[5] = I2C2RCV;        /* I2C : Read data */
      I2C2CONbits.ACKDT = 1;           /* set to NACK */
      I2C2CONbits.ACKEN = 1;           /* Start Acknowledge sequence */
      MCHP_I2C2_State++;
      break;

     case 105:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 106:
      MCHP_I2C24_Request = 0;          /* This I2C sequence could be Re-Enabled */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      break;

     case 107:                         /* Start a new I2C Sequence */
      I2C2CONbits.SEN = 1;             /* I2C: START  */
      MCHP_I2C2_State++;
      break;

     case 108:
      I2C2TRN = 0x3C;                  /* Send I2C Address : A7 A6 A5 A4 A3 A2 A1 A0 RW */
      MCHP_I2C2_State++;
      break;

     case 109:
      I2C2TRN = 0x00;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 110:
      I2C2TRN = 0x78;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 111:
      I2C2TRN = 0x00;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 112:
      I2C2TRN = 0x00;                  /* I2C : Write data */
      MCHP_I2C2_State++;
      break;

     case 113:
      I2C2CONbits.PEN = 1;             /* I2C: STOP */
      MCHP_I2C2_State++;
      break;

     case 114:
      MCHP_I2C25_Request = 0;          /* This I2C sequence could be Re-Enabled */
      MCHP_I2C2_State = 0;
      Continue = (MCHP_I2C2_Queue.head != MCHP_I2C2_Queue.tail);/* One next sequence queued, start it now */
      break;

     default:                          /* Sequence finished */
      MCHP_I2C2_State = 0;             /* Should never happend */
      break;
    }                                  /* End of switch case sequence*/
  } while (Continue == 1);             /* end do-While */
}                                      /* Enf of interrupt */
