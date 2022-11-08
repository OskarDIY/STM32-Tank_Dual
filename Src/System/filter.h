#ifndef __FILTER_H
#define __FILTER_H
#include <stdint.h>





#define IIR_SHIFT         8

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

typedef struct 
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2pData;

void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);


#endif //__FILTER_H
