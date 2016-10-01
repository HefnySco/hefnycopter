/*
 * KKv2pp.h
 *
 * Created: 8/13/2012 8:43:43 PM
 *  Author: hefny
 */ 


#ifndef KKV2PP_H_
#define KKV2PP_H_



//void Setup (void);
//void Loop(void);
//void RxGetChannels(void);

void Setup (void);
void LoopCalibration (void);
void LoopESCCalibration (void);
void Loop(void);
void MainLoop(void);
void HandleSticksForArming (void);
void HandleSticksAsKeys (void);
void ZEROMotors();


#endif /* KKV2PP_H_ */