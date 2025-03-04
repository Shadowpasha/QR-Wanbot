#ifndef SRC_SPID_SPID_H_
#define SRC_SPID_SPID_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	float *error_source;
	float prev_error;
	float sample_time;
	float *output;

	float integrator;
	float diffrentiator;

	float KP;
	float KI;
	float KD;
	float saturation;
	float integ_max;
	float KE;
	float KU;
	float tau;
}SPID_t;

void SPIDInit(SPID_t* SPIDX, float* error, float *output, float sample_time,float saturation,float intgerator_max,
		float KP, float KI, float KD, float KE, float KU);
void SPIDLoop(SPID_t* SPIDX);
void SPIDReset(SPID_t* SPIDX);

#ifdef __cplusplus
		}
#endif


#endif /* SRC_SPID_SPID_H_ */
