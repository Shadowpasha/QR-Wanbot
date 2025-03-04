#include "spid.h"

void SPIDInit(SPID_t* SPIDX, float* error, float *output, float sample_time,float saturation,float intgerator_max,
		float KP, float KI, float KD, float KE, float KU){

	SPIDX->saturation = saturation;
	SPIDX->integ_max = intgerator_max;
	SPIDX->sample_time = sample_time;
	SPIDX->KP = KP;
	SPIDX->KI = KI;
	SPIDX->KD = KD;
	SPIDX->KE = KE;
	SPIDX->KU = KU;

	SPIDX->error_source = error;
	SPIDX->integrator = 0.0;
	SPIDX->diffrentiator= 0.0;
	SPIDX->prev_error = 0.0;
	SPIDX->output = output;


}


void SPIDLoop(SPID_t* SPIDX){

	float error = *(SPIDX->error_source) * SPIDX->KE;


	SPIDX->integrator += ((error + SPIDX->prev_error) * 0.5  * (SPIDX->sample_time)) * SPIDX->KI;

	if(SPIDX->integrator > SPIDX->integ_max)
		SPIDX->integrator = SPIDX->integ_max;
	if(SPIDX->integrator < -SPIDX->integ_max)
		SPIDX->integrator = -SPIDX->integ_max;


	//Dervative and Low pass Filter using Measurement to prevent Impulse when on set point change or other not smooth activites
	SPIDX->diffrentiator = SPIDX->KD * ((error - SPIDX->prev_error)/(SPIDX->sample_time));

	float PID = SPIDX->KP * error + SPIDX->integrator + SPIDX->diffrentiator;

	if(PID > SPIDX->saturation)
		PID = SPIDX->saturation;
	if(PID < -SPIDX->saturation)
		PID = -SPIDX->saturation;

	SPIDX->prev_error = error;

	float output_buffer = (PID) * (SPIDX->KU);

	*(SPIDX->output) = output_buffer;

}

void SPIDReset(SPID_t* SPIDX){

	SPIDX->integrator = 0.0;
	SPIDX->diffrentiator= 0.0;
	SPIDX->prev_error = 0.0;

}
