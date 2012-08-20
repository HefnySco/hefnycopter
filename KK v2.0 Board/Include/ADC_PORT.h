/*
 * ADC.h
 *
 * Created: 8/14/2012 2:25:37 AM
 *  Author: hefny
 */ 


#ifndef ADC_H_
#define ADC_H_


class ADC_PORT {
	
	public:
		ADC_PORT();
		void Init();
		uint16_t adcRead(uint8_t channel);
		uint16_t adcGet(uint8_t channel);

};


#endif /* ADC_H_ */