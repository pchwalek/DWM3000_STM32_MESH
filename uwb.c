/*
 * uwb.c
 *
 *  Created on: Dec 15, 2021
 *      Author: patrick
 */

#include "uwb.h"

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <example_selection.h>
#include <shared_defines.h>
//#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_functions.h"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = { 5, /* Channel number. */
DWT_PLEN_128, /* Preamble length. Used in TX only. */
DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
9, /* TX preamble code. Used in TX only. */
9, /* RX preamble code. Used in RX only. */
1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
DWT_BR_6M8, /* Data rate. */
DWT_PHRMODE_STD, /* PHY header mode. */
DWT_PHRRATE_STD, /* PHY header rate. */
(129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
DWT_STS_MODE_OFF, /* Cipher disabled */
DWT_STS_LEN_64,/* Cipher length see allowed values in Enum dwt_sts_lengths_e */
DWT_PDOA_M0 /* PDOA mode off */
};

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32_t status_reg = 0;

/* Hold copy of frame length of frame received (if good), so reader can examine it at a breakpoint. */
static uint16_t frame_len = 0;

/* Hold copy of event counters so that it can be examined at a debug breakpoint. */
static dwt_deviceentcnts_t event_cnt;

/* Hold copy of diagnostics data so that it can be examined at a debug breakpoint. */
static dwt_rxdiag_t rx_diag;

/* Hold copy of accumulator data so that it can be examined at a debug breakpoint. See NOTE 2. */
#define ACCUM_DATA_LEN (3 * 2 * (3 + 3) + 1)
static uint8_t accum_data[ACCUM_DATA_LEN];

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer_range[RX_BUF_LEN];

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2800
//#define POLL_RX_TO_RESP_TX_DLY_UUS 1000

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 1000
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 1900
//#define FINAL_RX_TIMEOUT_UUS 700
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 0

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
volatile double distance_in;
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

//void UWB_InitiatorTask(void *argument) {
//
//}

osThreadId_t uwbResponderTaskHandle;

uint32_t avgReceiveStength(uint16_t num_of_samples, uint32_t timeout);
uint8_t isSignalStrengthGood(void);
uint32_t ds_twr_responder_session(void);

const osThreadAttr_t uwbResponderTask_attributes = { .name = "uwbResp",
		.attr_bits = osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem =
				NULL, .stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityNormal, .tz_module = 0, .reserved = 0 };

#define SIGNAL_STRENGTH_SAMPLE_SIZE 30
#define SIGNAL_STRENGTH_MEAS_TIMEOUT 10000 //10 seconds
#define SIGNAL_STRENGTH_THRESHOLD	120 //dBm

#define START_SINGLE_RESPONDER_SESSION	0x0100

static uint32_t flags;
#pragma GCC optimize("Ofast")

void UWB_ResponderTask(void *argument) {

//	flags = osThreadFlagsWait(START_SINGLE_RESPONDER_SESSION,
//	osFlagsWaitAny,
//	osWaitForever);

	flags = START_SINGLE_RESPONDER_SESSION;
	if ((flags &= START_SINGLE_RESPONDER_SESSION)
			== START_SINGLE_RESPONDER_SESSION) {
		// check if SPI is enabled and if not, enable

		// turn on radio
		/* Reset DW IC */
		reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

		Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

		while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
		{
		};

		// initialize and configure UWB radio
		if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
			while (1) {
			};
		}
		/* Configure DW IC. */
		if (dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
		{
			while (1) {
			};
		}

		// suspend scheduler while allowing interrupts
		vTaskSuspendAll();

//		if (isSignalStrengthGood()) {
		if (1) {
			//todo: send via OpenThread that this node is eligibile for a ranging session

			//start ranging session

	    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
	    dwt_configuretxrf(&txconfig_options);

	    /* Apply default antenna delay value. See NOTE 1 below. */
	    dwt_setrxantennadelay(RX_ANT_DLY);
	    dwt_settxantennadelay(TX_ANT_DLY);

	    // for debugging dev board only
	    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	    ds_twr_responder_session();
		}

		// suspend scheduler while allowing interrupts
		xTaskResumeAll();
	}
}

uint32_t ds_twr_responder_session(void){
  while (1)
  {
      dwt_setpreambledetecttimeout(0);
      /* Clear reception timeout to start next ranging process. */
      dwt_setrxtimeout(0);

      /* Activate reception immediately. */
      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
      { };

      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
      {
          uint32_t frame_len;

          /* Clear good RX frame event in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

          /* A frame has been received, read it into the local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
          if (frame_len <= RX_BUF_LEN)
          {
              dwt_readrxdata(rx_buffer_range, frame_len, 0);
          }

          /* Check that the frame is a poll sent by "DS TWR initiator" example.
           * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
          rx_buffer_range[ALL_MSG_SN_IDX] = 0;
          if (memcmp(rx_buffer_range, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
          {
              uint32_t resp_tx_time;
              int ret;

              /* Retrieve poll reception timestamp. */
              poll_rx_ts = get_rx_timestamp_u64();

              /* Set send time for response. See NOTE 9 below. */
              resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
              dwt_setdelayedtrxtime(resp_tx_time);

              /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
              dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
              dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
              /* Set preamble timeout for expected frames. See NOTE 6 below. */
              dwt_setpreambledetecttimeout(PRE_TIMEOUT);

              /* Write and send the response message. See NOTE 10 below.*/
              tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
              dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
              dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
              ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

              /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
              if (ret == DWT_ERROR)
              {
                  continue;
              }

              /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
              while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
              { };

              /* Increment frame sequence number after transmission of the response message (modulo 256). */
              frame_seq_nb++;

              if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
              {
                  /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                  /* A frame has been received, read it into the local buffer. */
                  frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                  if (frame_len <= RX_BUF_LEN)
                  {
                      dwt_readrxdata(rx_buffer_range, frame_len, 0);
                  }

                  /* Check that the frame is a final message sent by "DS TWR initiator" example.
                   * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                  rx_buffer_range[ALL_MSG_SN_IDX] = 0;
                  if (memcmp(rx_buffer_range, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                  {
                      uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                      uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                      double Ra, Rb, Da, Db;
                      int64_t tof_dtu;

                      /* Retrieve response transmission and final reception timestamps. */
                      resp_tx_ts = get_tx_timestamp_u64();
                      final_rx_ts = get_rx_timestamp_u64();

                      /* Get timestamps embedded in the final message. */
                      final_msg_get_ts(&rx_buffer_range[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                      final_msg_get_ts(&rx_buffer_range[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                      final_msg_get_ts(&rx_buffer_range[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                      /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                      poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                      resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                      final_rx_ts_32 = (uint32_t)final_rx_ts;
                      Ra = (double)(resp_rx_ts - poll_tx_ts);
                      Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                      Da = (double)(final_tx_ts - resp_rx_ts);
                      Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                      tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                      tof = tof_dtu * DWT_TIME_UNITS;
                      distance = tof * SPEED_OF_LIGHT;
                      distance_in = distance * 39.3701;
//                      /* Display computed distance on LCD. */
//                      sprintf(dist_str, "DIST: %3.2f m", distance);
//                      test_run_info((unsigned char *)dist_str);

                      /* as DS-TWR initiator is waiting for RNG_DELAY_MS before next poll transmission
                       * we can add a delay here before RX is re-enabled again
                       */
                      Sleep(RNG_DELAY_MS - 10);  //start couple of ms earlier
                  }
              }
              else
              {
                  /* Clear RX error/timeout events in the DW IC status register. */
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
              }
          }
      }
      else
      {
          /* Clear RX error/timeout events in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      }
  }
}

uint8_t isSignalStrengthGood(void) {

	/* Activate event counters. */
	dwt_configeventcounters(1);
	/* Enable IC diagnostic calculation and logging */
	dwt_configciadiag(1);


	uint32_t signal_strength;
	signal_strength = avgReceiveStength(SIGNAL_STRENGTH_SAMPLE_SIZE,
	SIGNAL_STRENGTH_MEAS_TIMEOUT);

	/* Disable event counters. */
	dwt_configeventcounters(0);
	/* Disable IC diagnostic calculation and logging */
	dwt_configciadiag(0);

	if (signal_strength < SIGNAL_STRENGTH_THRESHOLD) {
		return 1;
	} else {
		return 0;
	}

}

uint32_t avgReceiveStength(uint16_t num_of_samples, uint32_t timeout) {

	uint32_t avgIpatovPower = 0;
	uint32_t startTick = HAL_GetTick();

	uint8_t idx;
	for (idx = 0; idx < num_of_samples; idx++) {
		int i;

		/* TESTING BREAKPOINT LOCATION #1 */

		/* Clear local RX buffer, rx_diag structure and accumulator values to avoid having leftovers from previous receptions  This is not necessary
		 * but is included here to aid reading the data for each new frame.
		 * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
		 * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. All
		 * diagnostics data will also be available. */
		for (i = 0; i < FRAME_LEN_MAX; i++) {
			rx_buffer[i] = 0;
		}
		for (i = 0; i < ACCUM_DATA_LEN; i++) {
			accum_data[i] = 0;
		}

		memset(&rx_diag, 0, sizeof(rx_diag));

		/* Activate reception immediately. See NOTE 4 below. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll until a frame is properly received or an error/timeout occurs. See NOTE 5 below.
		 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
		 * function to access it. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
				& (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
			if ((HAL_GetTick() - startTick) > timeout)
				break;
		};

		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
			/* Clear good RX frame event in the DW IC status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

			/* A frame has been received, copy it to our local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
			if (frame_len <= FRAME_LEN_MAX) {
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Read diagnostics data. */
			dwt_readdiagnostics(&rx_diag);

			/* Read accumulator. See NOTES 2 and 6. */
			uint16_t fp_int = rx_diag.ipatovFpIndex >> 6;
			dwt_readaccdata(accum_data, ACCUM_DATA_LEN, (fp_int - 2));
		} else {
			/* Clear RX error events in the DW IC status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		}

		/* Read event counters. See NOTE 7. */
		dwt_readeventcounters(&event_cnt);

		avgIpatovPower += rx_diag.ipatovPower;
		if ((HAL_GetTick() - startTick) > timeout)
			break;
	}

	return avgIpatovPower / (idx + 1);
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 *    Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *    Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                   ^|P RMARKER|                                    - time of Poll TX/RX
 *                                   ^|R RMARKER|                    - time of Resp TX/RX
 *                                                      ^|R RMARKER| - time of Final TX/RX
 *
 *                       <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                    <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                      can turn around and reply)
 *
 *
 *                                        <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                  <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                    <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                      can turn around and reply)
 *
 * EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *            reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 * EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 200 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 38 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/

