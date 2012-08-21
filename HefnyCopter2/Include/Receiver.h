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
extern uint8_t RX_good;

#define RX_MODE_PWM		0
#define RX_MODE_CPPM	1





void rxInit(uint8_t mode);
void RX_StickCenterCalibrationInit(void);
 uint16_t RX_GetReceiverValues (uint8_t Channel);
void RX_StickCenterCalibration(void);




#endif /* RECEIVER_H_ */