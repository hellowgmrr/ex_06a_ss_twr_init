/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "lcd.h"
#include "port.h"

/* Example application name and version to display on LCD screen. */

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
static void ShowToLcd(const char *string)
{
    uint8 command;
    command = 0x2;
    writetoLCD(1, 0, &command);
    command = 0x1;
    writetoLCD(1, 0, &command);
    writetoLCD(40, 1, (const uint8 *)string);
}
//����һ����ʾ�е�������ʾ
static void ConShowToLcd(const char *string)
{
    writetoLCD(40, 1, (const uint8 *)string);
}


/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0,0,0,0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0,0,0,0,0,0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define Anc_12_Dis_IDX  11
#define RESP_MSG_TS_LEN 4
#define Anc0TxTo2 7
#define Anc2TxTo0 8
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 30
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 140
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 210

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static double Dis_Get_Tx(uint8 *dis_field);

char display[20] = {0};
int MeasureTimes=10;
int NowTimes = 1;
uint32 Anchor01Dis[10] = {0};
uint32 Anchor02Dis[10] = {0};
uint32 Anchor12Dis[10] = {0};
int Anchor2LoSuc = 0;


int main(void)
{
    double a03;
    double a_03;
	peripherals_init();
    int AnchorMode = 0;
    sprintf(display,"Anchor %d",AnchorMode);
    lcd_display_str(display);
    reset_DW1000();
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
    spi_set_rate_high();
    dwt_configure(&config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//�˴�����Ϊ����
    int i;
    /* Loop forever initiating ranging exchanges. */
    while(1)// (NowTimes <= MeasureTimes)
    {
      for(i=1;i<=3;i++)
     {
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[10] = i ;
        if(i==3)
        {
         tx_poll_msg[10] = Anc0TxTo2;
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is the expected response from the companion "SS TWR responder" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {

               if(i!=3){
                uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32 rtd_init, rtd_resp;

                /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                /* Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                /* Compute time of flight and distance. */
                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
               }
               else
               {
            	   distance =rx_buffer[20]*0.02;
            	   NowTimes++;
               }
                if(i==1)
                {
                	Anchor01Dis[NowTimes-1] = distance;
                }
                else if(i==2)
                {
                	Anchor02Dis[NowTimes-1] = distance;
                }
                else
                {
                	Anchor12Dis[NowTimes-1] = distance;
                }

                /* Display computed distance on LCD. */
                sprintf(dist_str, "DIST to%d: %3.2fm", i,distance);
                ShowToLcd(dist_str);
            }
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        dwt_rxreset();



//    	for(i=1;i<=0;i++)
//    	{
//    		dwt_rxenable(DWT_START_RX_IMMEDIATE);
//    	    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG |SYS_STATUS_ALL_RX_TO| SYS_STATUS_ALL_RX_ERR)))
//    	    {
//    	    	  if (status_reg & SYS_STATUS_RXFCG)
//    	    	    	    {
//    	    		             ShowToLcd("Rx ok");
//    	    	    	    	 uint32 frame_len;
//    	    	    	    	 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
//    	    	    	    	 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
//    	    	    	    	 if (frame_len <= RX_BUF_LEN)
//    	    	    	    	    {
//    	    	    	    	    dwt_readrxdata(rx_buffer, frame_len, 0);
//    	    	    	    	    }
//    	    	    	    	 rx_buffer[ALL_MSG_SN_IDX] = 0;
//    	    	    	    	 frame_seq_nb++;
//    	    	//    	    	 if ((memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)&&(rx_resp_msg[10] == 4))
//    	    	    	    	 if (rx_buffer[10] == 4)
//    	    	    	    	 {
//    	    	    	    	     i=10;
////    	    	    	    		 ShowToLcd("");
//    	    	    	    		 uint32 Anch12Distance;
//    	    	    	    		 Anch12Distance = rx_buffer[11];
//    	    	    	    		 distance = Anch12Distance;
//    	    	    	    		 Anchor12Dis[NowTimes-1] = distance;
//    	    	    	    		 sprintf(dist_str, "DIS 12:%3.2fm",distance);
//    	    	    	    		 ConShowToLcd(dist_str);
//    	    	    	    		 NowTimes++;
//    	    	    	    	 }
//    	    	    	    }
//    	    }//while
//    	}//for(i=1;i<=3;i++)
        /* Execute a delay between ranging exchanges. */
        sleep_ms(RNG_DELAY_MS);

      }
    }
 /* uint32 a_01,a_02,a_12;
    for(int a=0 ;a<10 ; a++)
    {
    uint32 a01 =Anchor01Dis[a];
    uint32 a02 =Anchor02Dis[a];
    uint32 a12 =Anchor12Dis[a];
    a_01 +=a01;
    a_02 +=a02;
    a_03 +=a03;

    }
   struct cooridinate
   {
       int x;
       int y;
   };
   struct cooridinate a0,a1,a2;
   a0.x=0;
   a0.y=0;
   a1.x=a_01/10;
   a1.y=0;
   a2.x=(pow(a_01,2)+pow(a_02,2)-pow(a_12,2))/(20*a_01);
   a2.y=sqrt(pow(a_02,2)-pow(a2.x,2))/10;
   while(1)
   {
    sprintf(dist_str, "a0 x: %3.2fm  a0 y: %3.2fm", a0.x,a0.y);
    ShowToLcd(dist_str);
    sleep_ms(200);
    sprintf(dist_str, "a1 x: %3.2fm  a1 y: %3.2fm", a1.x,a1.y);
    ShowToLcd(dist_str);
    sleep_ms(200);
    sprintf(dist_str, "a2 x: %3.2fm  a2 y: %3.2fm", a2.x,a2.y);
    ShowToLcd(dist_str);
    sleep_ms(200);
   }*/
}

static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

static double Dis_Get_Tx(uint8 *dis_field)
{
    int i,dis_int;
    double dis_double;
    for (i = 0; i <4; i++)
    {
        dis_int += dis_field[i]<<(i*8);
    }
    dis_double = dis_int*0.0001;
    return dis_double;
}
