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

#define IS_TX1_GOOD					((RX_Good & 0b00001111)==0) 
#define TX1_NOT_FOUND				(RX_Good  | 0b00000001)
#define TX1_DISCONNECTED			(RX_Good  | 0b00000010)
#define TX1_CONNECTED_ERR			(RX_Good  & 0b11110010) // should equal to ZERO
#define TX1_FOUND_ERR				(RX_Good  & 0b11110001) // should equal to ZERO

#define IS_TX2_GOOD					((RX_Good & 0b11110000)==0) 
#define TX2_NOT_FOUND				(RX_Good  | 0b00010000)
#define TX2_DISCONNECTED			(RX_Good  | 0b00100000)
#define TX2_CONNECTED_ERR			(RX_Good  & 0b00101111) // should equal to ZERO
#define TX2_FOUND_ERR				(RX_Good  & 0b00011111) // should equal to ZERO


void rxInit							(uint8_t mode);
void RX_StickCenterCalibrationInit	(uint8_t RXIndex);
void RX_StickCenterCalibration		(uint8_t RXIndex);
int16_t RX_GetReceiverThrottleValue (uint8_t RXIndex);
int16_t RX_GetReceiverValues		(uint8_t RXIndex,uint8_t Channel);
uint16_t RX_raw_GetReceiverValues	(uint8_t RXIndex, uint8_t Channel);
void RX_CopyLatestReceiverValues	(void);




#endif /* RECEIVER_H_ */