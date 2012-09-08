/*
 * Receiver.h
 *
 * Created: 8/18/2012 6:38:58 AM
 *  Author: hefny
 */ 


#ifndef RECEIVER_H_
#define RECEIVER_H_


//extern int16_t RX[RX_CHANNELS];
//extern uint16_t RX_raw[RX_CHANNELS];
uint8_t RX_Good;

#define RX_MODE_PWM		0
#define RX_MODE_CPPM	1

/*
#define IS_TX_GOOD				(RX_Good==0x00)
#define IS_TX_NOT_FOUND			((RX_Good & 0b00000001)==0b00000001)
#define IS_TX_DISCONNECTED		((RX_Good & 0b00000010)==0b00000010)
#define IS_TX_GOOD			     (RX_Good == 0x00)

*/

#define TX_GOOD					  0x0
#define TX_NOT_FOUND			 (RX_Good | 0b00000001)
#define TX_DISCONNECTED			 (RX_Good | 0b00000010)
#define TX_CONNECTED_ERR		 (RX_Good & 0b00000010) // should equal to ZERO
#define TX_FOUND_ERR			 (RX_Good & 0b00000001) // should equal to ZERO

void rxInit(uint8_t mode);
void RX_StickCenterCalibrationInit(void);
int16_t RX_GetReceiverThrottleValue ();
int16_t RX_GetReceiverValues (uint8_t Channel);
uint16_t RX_raw_GetReceiverValues (uint8_t Channel);
void RX_StickCenterCalibration(void);
void RX_CopyLatestReceiverValues (void);




#endif /* RECEIVER_H_ */