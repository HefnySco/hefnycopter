/*
 * ADC.h
 *
 * Created: 8/14/2012 2:25:37 AM
 *  Author: hefny
 */ 


#ifndef ADC_H_
#define ADC_H_


		void ADCPort_Init();
		uint16_t ADCPort_Read(uint8_t channel);
		uint16_t ADCPort_Get(uint8_t channel);




#endif /* ADC_H_ */