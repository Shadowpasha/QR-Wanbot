/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float position[4][3];      // [leg][x,y,z]
	float body_angles[4][3];   // [leg][pitch,roll,yaw]
	float joint_angles[4][3];  // [leg][hip,thigh,calf]
	float rotation[3];         // [pitch,roll,yaw]
	float req_rotation[3];     // Requested orientation
	float velocity[3];         // [vx, vy, v_yaw]
	float offsets[4][3];       // Leg offsets
	float config_pitch_offset;
	float config_roll_offset;
	float step_height;
	float hind_step_height;
	float step_length;
	int step_pace;
} RobotState_t;

RobotState_t state;
Srv_Drv_t pca9865;
MPU6050_t mpu6050;
SPID_t pid_pitch;
SPID_t pid_roll;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t pulses[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

float pitch_error, pitch_output, pitch_sum;
float roll_error, roll_output, roll_sum;

float body_height = 0.169;
float X_setpoint = 0.00, prev_X_setpoint = 0.0, Y_setpoint = 0.00, prev_Y_setpoint = 0.00, Turn_setpoint = 0.0, prev_Turn_setpoint = 0.0;
char tx_buffer[128];
char rx_buffer[128];
char num_conv[6];
char rx;

uint16_t  AD_RES_BUFFER[4];
float  force_sensors[4];

uint8_t rx_index = 0;
uint8_t pressed_button = 0;
uint32_t ticks;
enum buttons{STOP=0, A, B, C, D, X, Y, START, L1, R1};
enum modes{WALK = 0, TROT, GALLOP, AUTO, CALIBRATION};
uint8_t slider_speed = 0.0, slider_angle = 0.0;
int J1x = 0.0, J1y = 0.0, J2x = 0.0, J2y = 0.0;
uint8_t mode;
uint8_t disable_inv = 0;
uint8_t stair_climb_mode = 0;
float stair_hieght_offset = 0.00;

#define K_PITCH_BALANCE_X -0.0000 
#define K_ROLL_BALANCE_Y -0.0000  

#define FL_HIEGHT_OFFSET 0.001
#define FR_HIEGHT_OFFSET -0.001
#define BL_HIEGHT_OFFSET 0.002
#define BR_HIEGHT_OFFSET -0.007

#define front_left_side_offset 0.000
#define back_left_side_offset 0.000

#define front_right_side_offset 0.00
#define back_right_side_offset 0.000

// Force Balancing
float force_height_offsets[4] = {0,0,0,0};
float force_sensor_offsets[4] = {90.47f, 88.12f, 88.46f, 88.02f};
#define K_FORCE_BALANCER 0.00005f
#define MAX_FORCE_HEIGHT_OFFSET 0.020f
#define FORCE_TOLERANCE 3.0f


#define K_RAIBERT 0.007f  // Raibert heuristic gain
#define SWING_HEIGHT 0.038f

#define START_IN_CALIBRATION 0 // Set to 1 to start in Calibration Mode [90,0,90]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
		.name = "MainTask",
		.stack_size = 1000 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CalcTask */
osThreadId_t CalcTaskHandle;
const osThreadAttr_t CalcTask_attributes = {
		.name = "CalcTask",
		.stack_size = 1000 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void Square_Test(void);
void Walk_Forward_Test(void);

void Mainfunc(void *argument);
void CalcFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of MainTask */
	MainTaskHandle = osThreadNew(Mainfunc, NULL, &MainTask_attributes);

	/* creation of CalcTask */
	CalcTaskHandle = osThreadNew(CalcFunc, NULL, &CalcTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Mainfunc */
/**
 * @brief  Function implementing the MainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Mainfunc */
void Mainfunc(void *argument)
{
	/* USER CODE BEGIN Mainfunc */
	//	disable_inv = 1;

#if START_IN_CALIBRATION
	mode = CALIBRATION;
#else
	Rise();
	mode = TROT;
#endif
	// HAL_UART_Init is already called in main.c
	HAL_UART_Receive_IT(&huart1, &rx, 1);
	HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 4);

	for(;;)
	{
		ticks = HAL_GetTick();
		X_setpoint = (J1y * state.step_length) * 0.05 + prev_X_setpoint * 0.95;
		if(fabs(X_setpoint - 0.0) < 0.00002) X_setpoint = 0.0;
		prev_X_setpoint = X_setpoint;
		Y_setpoint = (J1x * state.step_length * 0.8)* 0.05 + prev_Y_setpoint * 0.95;
		if(fabs(Y_setpoint - 0.0) < 0.00002) Y_setpoint = 0.0;
		prev_Y_setpoint = Y_setpoint;
		Turn_setpoint = (J2x * state.step_length * 0.5) * 0.05 + prev_Turn_setpoint * 0.95;
		if(fabs(Turn_setpoint - 0.0) < 0.00002) Turn_setpoint = 0.0;
		prev_Turn_setpoint = Turn_setpoint;

		if (mode == CALIBRATION) {
			for(int i=0; i<4; i++) {
				state.joint_angles[i][0] = 1.5708f; // Hip 90
				state.joint_angles[i][1] = 3.14f;    // Upper 180
				state.joint_angles[i][2] = 1.5708f; // Lower 90
			}
		} else if(X_setpoint != 0.0 || Y_setpoint != 0.0 || state.rotation[2] != 0 || Turn_setpoint != 0.0){
			Gait_controller(ticks, X_setpoint, Y_setpoint, Turn_setpoint);
		}else{
			Stand();
		}

		if(stair_climb_mode == 0){
			state.req_rotation[0] = ((0.0025 * J2y) * 57.3248);
			state.req_rotation[1] = 0.0;
		}



		if(pressed_button == C){
			//			while(pressed_button == C);
			Walk_Forward_Test();
		}

		if(pressed_button == L1){
			state.rotation[2] = -0.12;
		}else if (pressed_button == R1){
			state.rotation[2] = 0.12;
		}else{
			state.rotation[2] = 0.0;
		}

		if (pressed_button == A){
			while(pressed_button == A);
			HiWave();
		}

		if (pressed_button == D){
			while(pressed_button == D);
			Square_Test();
			pressed_button = STOP;
		}

		//		if (pressed_button == C){
		//			while(pressed_button == C);
		//			Scratch();
		//		}

		if (pressed_button == X){
			while(pressed_button == X);
			stair_climb_mode += 1;
			if(stair_climb_mode>=2){
				stair_climb_mode=2;
			}

			sprintf(tx_buffer,"%d %d %s 0\r\n",(int)(-mpu6050.roll), (int)(mpu6050.pitch), stair_climb_mode == 2? "Flat Climb":"Inclined Climb");
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));
		}

		if (pressed_button == Y){
			while(pressed_button == Y);
			stair_climb_mode -= 1;
			if(stair_climb_mode<=0){
				stair_climb_mode=0;
			}
			sprintf(tx_buffer,"%d %d %s 0\r\n",(int)(-mpu6050.roll), (int)(mpu6050.pitch), stair_climb_mode == 1? "Inclined Climb":"Trot");
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));
		}

		if(stair_climb_mode == 2){
			state.config_pitch_offset = -0.15;
			stair_hieght_offset = 0.000;
			state.step_length = 0.00049;
			state.step_pace = 820;
			state.step_height = 0.055;
			state.hind_step_height = 0.060;

		}else if (stair_climb_mode == 1){
			state.config_pitch_offset = 2.95;
			stair_hieght_offset = 0.000;
			state.step_length = 0.00049;
			state.step_pace = 820;
			state.step_height = 0.055;
			state.hind_step_height = 0.060;

		}else{
			state.config_pitch_offset = -1.55; //1.55
			stair_hieght_offset = 0.0;
			state.step_length = 0.00038;
			state.step_pace = 650;
			state.step_height = 0.038;
			state.hind_step_height = 0.038;
		}

		if (pressed_button == B){
			//		HAL_UART_Transmit_IT(&huart1, data, strlen(data));
			while(pressed_button == B);
			if(mode == TROT){
				mode = WALK;
			}else if (mode == WALK){
				mode = TROT;
			}else if (mode == CALIBRATION){
				mode = TROT;
			}

			//			sprintf(data,"%d %d %s 0\r\n",(int)(-mpu6050.roll), (int)(mpu6050.pitch), mode? "Trot":"Walk");
			//			HAL_UART_Transmit_IT(&huart1, data, strlen(data));
		}

		//		sprintf(data,"%.3f %.3f %.3f %.3f\r\n", force_sensors[0], force_sensors[1], force_sensors[2], force_sensors[3]);
		//		sprintf(data,"waw\r\n");
		//
		//		HAL_UART_Transmit_IT(&huart1, data, strlen(data));
		//		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
		//		if(ticks == 0 && ticks == 2 && ticks == 3 && ticks == 5 && mode == TROT){

		//		sprintf(tx_buffer,"%f, %f\r\n",(-mpu6050.roll), (mpu6050.pitch));
		//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));

		static uint32_t last_print = 0;
		if (mode != CALIBRATION && ticks - last_print >= 200) {
			last_print = ticks;
			sprintf(tx_buffer,"F:%.1f %.1f %.1f %.1f | O:%.3f %.3f %.3f %.3f\r\n", 
					force_sensors[0], force_sensors[1], force_sensors[2], force_sensors[3],
					force_height_offsets[0], force_height_offsets[1], force_height_offsets[2], force_height_offsets[3]);
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));
		}
		HAL_Delay(1);
		//		}else if (mode == TROT){
		//			HAL_Delay(175);
		//		}else{
		//			HAL_Delay(250);
		//		}




		// Initilization for Servo Motors
		//		FL_angles[0] = 1.57;
		//		FL_angles[1] = 1.57;
		//		FL_angles[2] = 1.57;
		//
		//		FR_angles[0] = 1.57;
		//		FR_angles[1] = 1.57;
		//		FR_angles[2] = 1.57;
		//		//
		//		BL_angles[0] = 1.57;
		//		BL_angles[1] = 1.57;
		//		BL_angles[2] = 1.57;
		//
		//		BR_angles[0] = 1.57;
		//		BR_angles[1] = 1.57;
		//		BR_angles[2] = 1.57;

		//Up Down Code
		//		FL_position[2] = 0.13;
		//		HAL_Delay(500);
		//		FL_position[2] = 0.15;
		//		HAL_Delay(500);


		// Jumping Code
		//		FL_position[2] = 0.09;
		//		HAL_Delay(500);
		//		FL_position[2] = 0.15;
		//		HAL_Delay(150);
		//		FL_position[2] = 0.09;
		//		HAL_Delay(1000);

	}
	/* USER CODE END Mainfunc */
}

/* USER CODE BEGIN Header_CalcFunc */
/**
 * @brief Function implementing the CalcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CalcFunc */
void CalcFunc(void *argument)
{
	/* USER CODE BEGIN CalcFunc */
	//	disable_inv = 1;
	state.offsets[0][0] = 0.087; state.offsets[0][1] = 0.349; state.offsets[0][2] = 0.524;
	state.offsets[1][0] = -1.309; state.offsets[1][1] = -0.262; state.offsets[1][2] = 0.175;
	state.offsets[2][0] = -0.20; state.offsets[2][1] = -0.314; state.offsets[2][2] = 0.436;
	state.offsets[3][0] = -0.559; state.offsets[3][1] = 0.1921; state.offsets[3][2] = 0.18;

	state.config_pitch_offset = -0.55;
	state.config_roll_offset = -12.5;
	state.step_pace = 820;
	state.step_length = 0.00036;
	state.step_height = 0.036;
	state.hind_step_height = 0.036;

	HAL_Delay(300);
	ServoDriverInit(&pca9865, &hi2c1, SERVO_DRIVER_ADDRESS);
	MPUInit(&mpu6050, &hi2c2, MPU6050_DataRate_100Hz, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s, 0.01, 0.5);
	MPUSetOffsets(&mpu6050, -944, -600, -590, 1130, 16, 923);
	SPIDInit(&pid_pitch, &pitch_error, &pitch_output, 0.01, 0.8, 0.05, 0.10, 0.10, 0.0005, 1.0/30.0, 0.20);
	SPIDInit(&pid_roll, &roll_error, &roll_output, 0.01, 0.8, 0.05, 0.10, 0.10, 0.0005, 1.0/30.0, 0.20);
	//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	/* Infinite loop */
	for(;;)
	{
		//		osSemaphoreAcquire(sendSemaphoreHandle, osWaitForever);
#if START_IN_CALIBRATION


#else
		MPUReqAccGyro(&mpu6050);
		CompPitchRoll(&mpu6050);
		pitch_error = state.req_rotation[0] - (-mpu6050.roll + state.config_pitch_offset);  // Increase to move back
		roll_error = state.req_rotation[1] - (mpu6050.pitch +  state.config_roll_offset);  // decrease to tilt right
		if(fabsf(pitch_error) > 4.0){
			SPIDLoop(&pid_pitch);
			pitch_sum+=pitch_output;
			if (pitch_sum > 0.45) pitch_sum = 0.45;
			if (pitch_sum < -0.45) pitch_sum = -0.45;
			state.rotation[0]= pitch_sum;
		}
		if(fabsf(roll_error) > 4.0){
			SPIDLoop(&pid_roll);
			roll_sum+=roll_output;
			if (roll_sum > 0.45) roll_sum = 0.45;
			if (roll_sum < -0.45) roll_sum = -0.45;
			state.rotation[1]= roll_sum;

		}



		//		if(fabsf(state.rotation[0] - pitch_sum) >= 0.05)

		//		if(fabsf(state.rotation[1] - roll_sum) >= 0.05)


		//		--- Force Balancing Logic ---
		//		float total_force = 0;
		//		for (int i = 0; i < 4; i++) {
		//			total_force += force_sensors[i];
		//		}
		//		float avg_force = total_force / 4.0f;
		//
		//		float offset_sum = 0;
		//		for (int i = 0; i < 4; i++) {
		//			float force_error = force_sensors[i] - avg_force;
		//
		//			// Only adjust if error is outside tolerance (deadband)
		//			if (fabsf(force_error) > FORCE_TOLERANCE) {
		//				force_height_offsets[i] -= K_FORCE_BALANCER * force_error;
		//			}
		//			offset_sum += force_height_offsets[i];
		//		}
		//
		//		// Constrain sum of offsets to 0 to prevent the whole robot from sinking or rising
		//		float avg_offset = offset_sum / 4.0f;
		//		for (int i = 0; i < 4; i++) {
		//			force_height_offsets[i] -= avg_offset;
		//
		//			// Clamp the offset to prevent excessive leg movement
		//			if (force_height_offsets[i] > MAX_FORCE_HEIGHT_OFFSET) force_height_offsets[i] = MAX_FORCE_HEIGHT_OFFSET;
		//			if (force_height_offsets[i] < -MAX_FORCE_HEIGHT_OFFSET) force_height_offsets[i] = -MAX_FORCE_HEIGHT_OFFSET;
		//		}
		inverse_kinematics_all();
#endif

		load_angles();
		ServoDriverSetOnOff_Multi(&pca9865, 0, 12, pulses);
		HAL_Delay(5);
	}
	/* USER CODE END CalcFunc */
}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	if(rx != '\n' && rx_index < 127){
		rx_buffer[rx_index] = rx;
		rx_index +=1;
	}else{
		rx_buffer[rx_index] = rx;
		rx_index = 0;

		switch(rx_buffer[0]){
		case 'A' :
			pressed_button = A;
			break;
		case 'B' :
			pressed_button = B;
			break;
		case 'C' :
			pressed_button = C;
			break;
		case 'D' :
			pressed_button = D;
			break;
		case 'X' :
			pressed_button = X;
			break;
		case 'Y' :
			pressed_button = Y;
			break;
		case 'R' :
			pressed_button = R1;
			break;
		case 'K' :
			mode = CALIBRATION;
			sprintf(tx_buffer, "Entering CALIBRATION Mode\r\n");
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));
			break;
		case 'O' : // Offset command: O <leg> <joint> <value_deg>
		{
			int leg = 0, joint = 0;
			float val = 0.0f;
			// Format: O 0 1 5.5
			if (sscanf(rx_buffer, "O %d %d %f", &leg, &joint, &val) == 3) {
				if (leg >= 0 && leg < 4 && joint >= 0 && joint < 3) {
					state.offsets[leg][joint] = val * 0.0174533f; // Convert degrees to radians
					sprintf(tx_buffer, "Leg %d Offsets(rad): H:%.3f U:%.3f L:%.3f\r\n", 
							leg, state.offsets[leg][0], state.offsets[leg][1], state.offsets[leg][2]);
					HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));
				}
			}
		}
		break;
		case 'S' :
			if(rx_buffer[1] == 'T'){
				pressed_button = START;
			}else if(rx_buffer[1] == '1'){

				uint8_t joy_index = 2;
				memset(num_conv,'_',6);
				while( rx_buffer[joy_index] != '\n'){
					num_conv[joy_index] = rx_buffer[joy_index];
					joy_index+=1;
				}
				slider_speed = (uint8_t)(atoi(num_conv));

			}else if(rx_buffer[1] == '2'){

				uint8_t joy_index = 2;
				memset(num_conv,'_',6);
				while( rx_buffer[joy_index] != '\n'){
					num_conv[joy_index] = rx_buffer[joy_index];
					joy_index+=1;
				}
				slider_angle = (uint8_t)(atoi(num_conv));

			}else{
				pressed_button = STOP;
				J1x = 0.0;
				J1y = 0.0;
				J2y = 0.0;
				J2x = 0.0;
			}
			break;
		case 'J':
			if(rx_buffer[1] == '1'){
				uint8_t joy_index = 2;
				uint8_t second_index = 0;
				memset(num_conv,'_',6);

				while( rx_buffer[joy_index] != ' '){
					num_conv[second_index] = rx_buffer[joy_index];
					joy_index+=1;
					second_index+=1;
				}

				J1x = (atoi(num_conv));
				//				if(J1x > 100) J1x = 100;
				//				if(J1x < -100) J1x = -100;

				memset(num_conv,'_',6);

				second_index = 0;
				joy_index+=1;

				while( rx_buffer[joy_index] != '\n'){
					num_conv[second_index] = rx_buffer[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J1y = (atoi(num_conv));
				//				if(J1y > 100) J1y = 100;
				//				if(J1y < -100) J1y = -100;

			}else if(rx_buffer[1] == '2'){
				uint8_t joy_index = 2;
				uint8_t second_index = 0;
				memset(num_conv,'_',6);

				while( rx_buffer[joy_index] != ' '){
					num_conv[second_index] = rx_buffer[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J2x = (atoi(num_conv));
				//				if(J2x > 100) J2x = 100;
				//				if(J2x < -100) J2x = -100;

				memset(num_conv,'_',6);
				second_index = 0;
				joy_index+=1;

				while( rx_buffer[joy_index] != '\n'){
					num_conv[second_index] = rx_buffer[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J2y = (atoi(num_conv));
				//				if(J2y > 100) J2y = 100;
				//				if(J2y < -100) J2y = -100;
			}
			break;
		}
		memset(rx_buffer,'0',20);
	}

	HAL_UART_Receive_IT(&huart1, &rx, 1);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// Conversion Complete & DMA Transfer Complete As Well
	// So The AD_RES_BUFFER Is Now Updated
	force_sensors[0] = ((float)((AD_RES_BUFFER[0] << 4) - 35000)*0.003f) - force_sensor_offsets[0];  // ADC CH1
	force_sensors[1] = ((float)((AD_RES_BUFFER[1] << 4) - 35000)*0.003f) - force_sensor_offsets[1];  // ADC CH2
	force_sensors[2] = ((float)((AD_RES_BUFFER[2] << 4) - 35000)*0.003f) - force_sensor_offsets[2];  // ADC CH3
	force_sensors[3] = ((float)((AD_RES_BUFFER[3] << 4) - 35000)*0.003f) - force_sensor_offsets[3];  // ADC CH4
}

float clamp(float value, float min, float max){
	if(value < min) return min;
	if(value > max) return max;
	return value;
}

void inverse_leg_kinematics(float position[], float angles[], uint8_t leg_index, float rotation[]){

	if(position[0] == 0.0) position[0] = -0.00001;
	if(position[1] == 0.0) position[1] = -0.00001;

	//	if(position[2] >= 0.18) position[2] = 0.18;


	float pitch_offset = sin(rotation[0]) * (body_length/2);
	float x_pitch_offset = ((position[2] - pitch_offset) * tan(rotation[0])  * 0.2);

	float roll_offset = sin(rotation[1]) * (body_width/2);
	float y_roll_offset = ((position[2] - roll_offset) * tan(rotation[1]) * 0.2);
	float current_yaw,new_width, new_length, H_leg, new_yaw,new_height,new_x,new_y,H1y,H2y,phi, phi2,theta,Hx,trident ;

	switch(leg_index){
	case 1:
		new_height = position[2] - pitch_offset + roll_offset + FL_HIEGHT_OFFSET + force_height_offsets[0];
		new_y = -position[1] + y_roll_offset - Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset + front_left_side_offset;
		H_leg = hypot(-body_width/2.0 + new_y,body_length/2.0 + new_x);
		current_yaw = atan2(body_length/2.0 + new_x, -body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length - body_length/2.0;
		new_y = new_width + body_width/2.0 - balance_offset;
		break;
	case 2:
		new_height = position[2] - pitch_offset - roll_offset +  FR_HIEGHT_OFFSET + force_height_offsets[1];
		new_y = position[1] - y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset + front_right_side_offset;
		H_leg = hypot(body_width/2.0 + new_y,body_length/2.0 + new_x);
		current_yaw = atan2(body_length/2.0 + new_x, body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length - body_length/2.0;
		new_y = new_width - body_width/2.0 - balance_offset;
		break;
	case 3:
		new_height = position[2] + pitch_offset + roll_offset + BL_HIEGHT_OFFSET + force_height_offsets[2];
		new_y = -position[1] + y_roll_offset - Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset - hind_leg_offest + back_left_side_offset;
		H_leg = hypot(-body_width/2.0 + new_y,-body_length/2.0 + new_x);
		current_yaw = atan2(-body_length/2.0 + new_x, -body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length + body_length/2.0;
		new_y = new_width + body_width/2.0 - balance_offset;
		break;
	case 4:
		new_height = position[2] + pitch_offset - roll_offset + BR_HIEGHT_OFFSET + force_height_offsets[3];
		new_y = position[1] - y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset - hind_leg_offest + back_right_side_offset;
		H_leg = hypot(body_width/2.0 + new_y,-body_length/2.0 + new_x);
		current_yaw = atan2(-body_length/2.0 + new_x, body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length + body_length/2.0;
		new_y = new_width - body_width/2.0 - balance_offset;
		break;
	}

	H1y = hypot(new_y,new_height);
	H2y = sqrtf(fmaxf(0.0f, pow(H1y,2)- pow(Y_offset,2)));
	phi = atan2(new_y,new_height);

	float arg_phi2 = Y_offset/H2y;
	if (H2y < 0.00001f) arg_phi2 = 1.0f;
	phi2 = acos(clamp(arg_phi2, -1.0f, 1.0f));
	angles[0] = phi  + phi2;

	theta = atan2(new_x,H2y);
	Hx = hypot(new_x,H2y);

	// --- Explicit Reach Clamping ---
	// Prevent leg from snapping straight or folding too tight
	float max_reach = upper_leg_len + lower_leg_len - 0.001f; // 1mm buffer
	float min_reach = fabsf(upper_leg_len - lower_leg_len) + 0.001f;

	if (Hx > max_reach) Hx = max_reach;
	if (Hx < min_reach) Hx = min_reach;


	float arg_trident = (pow(Hx,2) + pow(upper_leg_len,2) - pow(lower_leg_len,2)) / (2 * Hx * upper_leg_len);
	if (Hx < 0.00001f) arg_trident = 1.0f;
	trident =  acos(clamp(arg_trident, -1.0f, 1.0f));

	angles[1] = trident - theta + 1.5707;

	float arg_knee = (pow(lower_leg_len,2) + pow(upper_leg_len,2) - pow(Hx,2)) / (2 * lower_leg_len * upper_leg_len);
	angles[2] = acos(clamp(arg_knee, -1.0f, 1.0f));
	//	adjust_servo_angles(angles,leg_index);

}


void inverse_kinematics_all(void){
	inverse_leg_kinematics(state.position[0], state.joint_angles[0], 1, state.body_angles[0]);
	inverse_leg_kinematics(state.position[1], state.joint_angles[1], 2, state.body_angles[1]);
	inverse_leg_kinematics(state.position[2], state.joint_angles[2], 3, state.body_angles[2]);
	inverse_leg_kinematics(state.position[3], state.joint_angles[3], 4, state.body_angles[3]);
}

void load_leg_position(float positions[], float x, float y, float z, float angles[], float p, float r, float yaw){
	positions[0] = x;
	positions[1] = y;
	positions[2] = z;

	angles[0] = p;
	angles[1] = r;
	angles[2] = yaw;
}

void load_angles(){

	pulses[0] = (650 - angle_to_pulse(state.joint_angles[0][0] + state.offsets[0][0])); 
	pulses[1] = (650 - angle_to_pulse(state.joint_angles[0][1] + state.offsets[0][1]));
	pulses[2] = angle_to_pulse(state.joint_angles[0][2] + state.offsets[0][2]);

	pulses[3] = angle_to_pulse(state.joint_angles[1][0] + state.offsets[1][0]);
	pulses[4] = angle_to_pulse(state.joint_angles[1][1] + state.offsets[1][1]) ;
	pulses[5] = (650 - angle_to_pulse(state.joint_angles[1][2] + state.offsets[1][2]));

	pulses[6] = angle_to_pulse(state.joint_angles[2][0] + state.offsets[2][0]);
	pulses[7] = (650 - angle_to_pulse(state.joint_angles[2][1] + state.offsets[2][1]));
	pulses[8] = angle_to_pulse(state.joint_angles[2][2] + state.offsets[2][2]);

	pulses[9] = (650 - angle_to_pulse(state.joint_angles[3][0] + state.offsets[3][0]));
	pulses[10] = angle_to_pulse(state.joint_angles[3][1] + state.offsets[3][1]) ;
	pulses[11] = (650 - angle_to_pulse(state.joint_angles[3][2] + state.offsets[3][2]));

}

void Stand(){
	load_leg_position(state.position[0], 0.0, 0.0, body_height, state.body_angles[0], state.rotation[0], state.rotation[1], state.rotation[2]);
	load_leg_position(state.position[1], 0.0, 0.0, body_height, state.body_angles[1], state.rotation[0], state.rotation[1], state.rotation[2]);
	load_leg_position(state.position[2], 0.0, 0.0, body_height, state.body_angles[2], state.rotation[0], state.rotation[1], state.rotation[2]);
	load_leg_position(state.position[3], 0.0, 0.0, body_height, state.body_angles[3], state.rotation[0], state.rotation[1], state.rotation[2]);
}

void Gait_controller (uint32_t ticks, float x_setpoint, float y_setpoint, float turn_setpoint){

	if(mode == WALK){
		leg_cycle(state.position[0], 1, WALK, ticks, x_setpoint + turn_setpoint, y_setpoint, state.body_angles[0], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[1], 2, WALK, ticks, x_setpoint - turn_setpoint, y_setpoint, state.body_angles[1], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[2], 3, WALK, ticks, x_setpoint + turn_setpoint, y_setpoint, state.body_angles[2], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[3], 4, WALK, ticks, x_setpoint - turn_setpoint, y_setpoint, state.body_angles[3], state.rotation[0], state.rotation[1], state.rotation[2]);

	}else if(mode == TROT){
		leg_cycle(state.position[0], 1, TROT, ticks, x_setpoint + turn_setpoint, y_setpoint, state.body_angles[0], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[1], 2, TROT, ticks, x_setpoint - turn_setpoint, y_setpoint, state.body_angles[1], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[2], 3, TROT, ticks, x_setpoint + turn_setpoint, y_setpoint, state.body_angles[2], state.rotation[0], state.rotation[1], state.rotation[2]);
		leg_cycle(state.position[3], 4, TROT, ticks, x_setpoint - turn_setpoint, y_setpoint, state.body_angles[3], state.rotation[0], state.rotation[1], state.rotation[2]);
	}
	else if (mode == GALLOP){
		//
		//		switch(ticks){
		//
		//		case 0:
		//			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height-step_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height-step_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		case 1:
		//			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		case 2:
		//			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height - step_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height - step_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		case 3:
		//			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height - step_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height - step_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		case 4:
		//			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		case 5:
		//			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height - step_height, FL_body_angles, 0, 0, 0);
		//			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height, FR_body_angles, 0, 0, 0);
		//			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height, BL_body_angles, 0, 0, 0);
		//			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height - step_height, BR_body_angles, 0, 0, 0);
		//			break;
		//		}
	}
}


int angle_to_pulse(float angle){
	int pulse;

	pulse = (uint16_t)(angle * 90.7549) + 115;

	if (pulse > 535) pulse = 535;
	if (pulse < 115) pulse = 115;

	return pulse;
}


void Rise(){

	float position = 0.04;
	for ( int j = 0; j < 100; j++){
		position += 0.0013;
		state.position[0][2] = position;
		state.position[1][2] = position;
		state.position[2][2] = position;
		state.position[3][2] = position;
		HAL_Delay(10);
	}
}


// --- Configuration ---
// -- TROT SPECIFIC --
#define TROT_SWING_END 0.49

// -- WALK SPECIFIC --
#define WALK_SWING_END 0.25

// --- Bezier Trajectory Functions ---

/**
 * @brief 5th-order Bezier curve for smooth swing trajectories.
 * @param t Progress (0.0 to 1.0)
 * @param p0 Start point
 * @param p1 Control point 1
 * @param p2 Control point 2
 * @param p3 Control point 3
 * @param p4 Control point 4
 * @param p5 End point
 */
float bezier_5th(float t, float p0, float p1, float p2, float p3, float p4, float p5) {
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;
	float mt = 1.0f - t;
	float mt2 = mt * mt;
	float mt3 = mt2 * mt;
	float mt4 = mt3 * mt;
	float mt5 = mt4 * mt;

	return p0*mt5 + 5*p1*mt4*t + 10*p2*mt3*t2 + 10*p3*mt2*t3 + 5*p4*mt*t4 + p5*t5;
}

/**
 * @brief Simple 1D Bezier with defaults for swing height.
 */
float swing_bezier_z(float t, float height) {
	// Standard swing profile: [0, 0, height*1.5, height*1.5, 0, 0]
	return bezier_5th(t, 0.0f, 0.0f, height*2.0f, height*2.0f, 0.0f, 0.0f);
}

void leg_cycle(float positions[], uint8_t leg, uint8_t mode, uint32_t ticks, float x_setpoint, float y_setpoint, float angles[], float p, float r, float yaw) {
	float adjusted_time_in_cycle = 0.0f;
	float stance_time = (float)state.step_pace / 1000.0f * (1.0f - TROT_SWING_END);

	if (mode == TROT) {
		adjusted_time_in_cycle = (double)(ticks % state.step_pace) / (float)state.step_pace;

		if (leg == 2 || leg == 3) {
			adjusted_time_in_cycle = fmod(adjusted_time_in_cycle + 0.5, 1.0);
		}

		if (adjusted_time_in_cycle < TROT_SWING_END) {
			// SWING PHASE
			float swing_progress = adjusted_time_in_cycle / TROT_SWING_END;

			// --- Raibert Heuristic Foot Placement ---
			// Predict touchdown location based on current "velocity" (setpoint)
			float touchdown_x = x_setpoint + (stance_time / 2.0f) * x_setpoint * K_RAIBERT;
			float touchdown_y = y_setpoint + (stance_time / 2.0f) * y_setpoint * K_RAIBERT;

			positions[0] = bezier_5th(swing_progress, -x_setpoint, -x_setpoint, 0, 0, touchdown_x, touchdown_x);
			positions[1] = bezier_5th(swing_progress, -y_setpoint, -y_setpoint, 0, 0, touchdown_y, touchdown_y);

			float base_z = swing_bezier_z(swing_progress, state.step_height);
			positions[2] = body_height - base_z;
			angles[2] = generate_smooth_xy_trajectory(-yaw, yaw, swing_progress);

		} else {
			// STANCE PHASE
			float raw_progress = (adjusted_time_in_cycle - TROT_SWING_END) / (1.0f - TROT_SWING_END);
			float stance_progress = fmaxf(0.0f, raw_progress);

			positions[0] = x_setpoint + (-x_setpoint - x_setpoint) * stance_progress; 
			positions[1] = y_setpoint + (-y_setpoint - y_setpoint) * stance_progress;
			positions[2] = body_height;
			angles[2] = yaw + (-yaw - yaw) * stance_progress;
		}

	} else if (mode == WALK) {
		adjusted_time_in_cycle = (double)(ticks % 2000) / 2000.0;

		if (leg == 2) adjusted_time_in_cycle = fmod(adjusted_time_in_cycle + 0.25, 1.0);
		else if (leg == 3) adjusted_time_in_cycle = fmod(adjusted_time_in_cycle + 0.5, 1.0);
		else if (leg == 4) adjusted_time_in_cycle = fmod(adjusted_time_in_cycle + 0.75, 1.0);

		if (adjusted_time_in_cycle < WALK_SWING_END) {
			// SWING PHASE
			float swing_progress = adjusted_time_in_cycle / WALK_SWING_END;

			positions[0] = generate_smooth_xy_trajectory(-x_setpoint, x_setpoint, swing_progress);
			positions[1] = generate_smooth_xy_trajectory(-y_setpoint, y_setpoint, swing_progress);

			float base_z;
			if (stair_climb_mode > 0) {
				base_z = generate_step_z_trajectory(swing_progress, state.step_height);
			} else {
				base_z = generate_smooth_z_trajectory(swing_progress, state.step_height);
			}

			positions[2] = body_height - base_z;
			angles[2] = 0;

		} else {
			// STANCE PHASE
			float raw_progress = (adjusted_time_in_cycle - WALK_SWING_END) / (1.0f - WALK_SWING_END);
			float stance_progress = fmaxf(0.0f, raw_progress);

			positions[0] = x_setpoint + (-x_setpoint - x_setpoint) * stance_progress; // Linear stance
			positions[1] = y_setpoint + (-y_setpoint - y_setpoint) * stance_progress;
			positions[2] = body_height;
			angles[2] = 0;
		}
	}
	angles[0] = p;
	angles[1] = r;
}

float generate_smooth_xy_trajectory(float min, float max, float t){
	float s_curve = t - (1.0f / 6.283185f) * sinf(6.283185f * t);
	return min + (max - min) * s_curve;
}

float generate_smooth_z_trajectory(float t, float height) {
	return height * (1.0f - cosf(6.283185f * t)) / 2.0f;
}

float generate_step_z_trajectory(float t, float max_height) {
	// Phase 1: Lift (0.0 to 0.3) - Move Z from body_height (drop 0) up to peak (drop max_height)
	if (t <= 0.2) {
		float t_rel = t / 0.2;
		return max_height * t_rel;
	}
	// Phase 2: Hold (0.2 to 0.8) - Maintain peak height while moving forward
	else if (t <= 0.8) {
		return max_height;
	}
	// Phase 3: Descent (0.8 to 1.0) - Move Z from peak (drop max_height) back down to ground (drop 0)
	else {
		float t_rel = (t - 0.8) / 0.2;
		return max_height * (1.0 - t_rel);
	}
}


void Walk_Forward_Test(void){
	float speed_setpoint = 0.045; 
	float estimated_velocity = (2.0 * speed_setpoint) * (1000.0 / (float)state.step_pace);
	uint32_t duration = 6000; // Run for 6 seconds

	uint32_t start_time = HAL_GetTick();

	while((HAL_GetTick() - start_time) < duration){
		ticks = HAL_GetTick();
		Gait_controller(ticks, speed_setpoint, 0.0, 0.0);

		// Print IMU data
		sprintf(tx_buffer,"%f, %f\r\n",(-mpu6050.roll), (mpu6050.pitch));
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer));

		HAL_Delay(20);
	}
	Stand();
	HAL_Delay(500);

	pressed_button = STOP;
}


void Square_Test(void){

	float speed_setpoint = 0.035; // 3.5cm half-stride -> 7cm stride
	// Estimate velocity: Stride * Frequency
	// Frequency = 1000 ms / step_pace
	// Stride = 2 * speed_setpoint
	// Velocity = (2 * 0.035) * (1000 / 820) approx 0.085 m/s

	float estimated_velocity = (2.0 * speed_setpoint) * (1000.0 / (float)state.step_pace);
	uint32_t duration = (uint32_t)((1.0 / estimated_velocity) * 1000); // Duration for 1m in ms

	uint32_t start_time = 0;

	// 1. Move Forward (+X)
	start_time = HAL_GetTick();
	while((HAL_GetTick() - start_time) < duration){
		ticks = HAL_GetTick();
		Gait_controller(ticks, speed_setpoint, 0.0, 0.0);
		HAL_Delay(1);
	}
	Stand();
	HAL_Delay(500);

	// 2. Move Right (-Y) (Assuming Y is Left)
	start_time = HAL_GetTick();
	while((HAL_GetTick() - start_time) < duration){
		ticks = HAL_GetTick();
		Gait_controller(ticks, 0.0, -speed_setpoint, 0.0);
		HAL_Delay(1);
	}
	Stand();
	HAL_Delay(500);

	// 3. Move Backward (-X)
	start_time = HAL_GetTick();
	while((HAL_GetTick() - start_time) < duration){
		ticks = HAL_GetTick();
		Gait_controller(ticks, -speed_setpoint, 0.0, 0.0);
		HAL_Delay(1);
	}
	Stand();
	HAL_Delay(500);

	// 4. Move Left (+Y)
	start_time = HAL_GetTick();
	while((HAL_GetTick() - start_time) < duration){
		ticks = HAL_GetTick();
		Gait_controller(ticks, 0.0, speed_setpoint, 0.0);
		HAL_Delay(1);
	}
	Stand();
	HAL_Delay(1000);

}


void HiWave(){
	disable_inv = 1;

	state.position[2][0] = 0.02; // BL X
	state.position[3][0] = 0.02; // BR X
	state.position[0][0] = -0.02; // FL X

	state.position[2][2] = 0.16; // BL Z
	state.position[3][2] = 0.16; // BR Z

	state.position[0][2] = 0.21; // FL Z
	state.position[1][2] = 0.21; // FR Z

	HAL_Delay(1000);

	for (int i = 0; i < 10; i++){
		state.position[1][2] -= 0.01;   // FR Z
		state.position[1][0] += 0.008;  // FR X
		state.position[0][1] = -0.008; // FL Y
		HAL_Delay(100);
	}

	for (int i = 0; i < 6; i++){
		HAL_Delay(400);
		state.position[1][1] = -0.005; // FR Y
		HAL_Delay(400);
		state.position[1][1] = -0.01;  // FR Y
	}

	disable_inv = 0;
}



void Scratch(){
	disable_inv = 1;

	state.position[2][0] = 0.02; // BL X
	state.position[3][0] = 0.02; // BR X
	state.position[0][0] = -0.02; // FL X

	state.position[2][2] = 0.16; // BL Z
	state.position[3][2] = 0.16; // BR Z

	state.position[0][2] = 0.21; // FL Z
	state.position[1][2] = 0.21; // FR Z

	HAL_Delay(1000);

	state.position[1][2] = 0.06; // FR Z
	state.position[1][0] = 0.05; // FR X

	for (int i = 0; i < 15; i++){
		HAL_Delay(100);
		state.position[1][1] = -0.005; // FR Y
		HAL_Delay(100);
		state.position[1][1] = -0.01;  // FR Y
	}

	disable_inv = 0;
}

/* USER CODE END Application */

