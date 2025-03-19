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
Srv_Drv_t pca9865;
MPU6050_t mpu6050;
SPID_t pid_pitch;
SPID_t pid_roll;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t pulses[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
float FL_offsets[3];
float FR_offsets[3];
float BL_offsets[3];
float BR_offsets[3];
//position
float FL_position[3]  = {0.0,0.0,0.03};
float FR_position[3]  = {0.0,0.0,0.03};
float BL_position[3]  = {0.0,0.0,0.03};
float BR_position[3]  = {0.0,0.0,0.03};

//body_angles
float FL_body_angles[3]  = {0.0,0.0,0.0};
float FR_body_angles[3]  = {0.0,0.0,0.0};
float BL_body_angles[3]  = {0.0,0.0,0.0};
float BR_body_angles[3]  = {0.0,0.0,0.0};

//prev_position
float smoothed_FL_position[3]  = {0.0,0.0,0.03};
float smoothed_FR_position[3]  = {0.0,0.0,0.03};
float smoothed_BL_position[3]  = {0.0,0.0,0.03};
float smoothed_BR_position[3]  = {0.0,0.0,0.03};

//prev_position
float prev_FL_position[3]  = {0.0,0.0,0.03};
float prev_FR_position[3]  = {0.0,0.0,0.03};
float prev_BL_position[3]  = {0.0,0.0,0.03};
float prev_BR_position[3]  = {0.0,0.0,0.03};

//angles
float FL_angles[3];
float FR_angles[3];
float BL_angles[3];
float BR_angles[3];

float body_rotation[3] = {0.000,0.000,0.0000};
float req_body_rotation[3] = {0.000,0.000,0.000};

float pitch_error, pitch_output, pitch_sum;
float roll_error, roll_output, roll_sum;

float step_height = 0.042, step_length = 0.00028;
float x_translation = 0.01, y_translation = 0.021;
float body_height = 0.165;
float X_setpoint = 0.00, Y_setpoint = 0.00;
char data[30];
char num_conv[6];
char rx;
uint8_t rx_index = 0;
uint8_t pressed_button = 0;
uint8_t ticks;
enum buttons{STOP=0, A, B, C, D, X, Y, START, L1, R1};
enum modes{WALK = 0, TROT, GALLOP, AUTO};
uint8_t slider_speed = 0.0, slider_angle = 0.0;
int J1x = 0.0, J1y = 0.0, J2x = 0.0, J2y = 0.0;
uint8_t mode;
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
	Rise();
	mode = TROT;
	HAL_UART_Init(&huart1);
	HAL_UART_Receive_IT(&huart1, &rx, 1);
	for(;;)
	{
		X_setpoint = J1y * step_length;
		Y_setpoint = J1x * step_length * 0.75;
		if(X_setpoint != 0.0 || Y_setpoint != 0.0 || body_rotation[2] != 0){
			Gait_controller(ticks, X_setpoint, Y_setpoint);
			//			osSemaphoreRelease(sendSemaphoreHandle);
			ticks+=1;
			if ( mode == TROT){
				if(ticks > 5)
					ticks = 0;
			}else if (mode == WALK){
				if(ticks > 10)
					ticks = 0;
			}
		}else{
			Stand();
		}

		req_body_rotation[0] = (0.0025 * J2y) * 57.3248;
		req_body_rotation[1] = (0.0025 * J2x) * 57.3248;

		if(pressed_button == L1){
			body_rotation[2] = -0.17;
		}else if (pressed_button == R1){
			body_rotation[2] = 0.17;
		}else{
			body_rotation[2] = 0.0;
		}

		//		if (pressed_button == A){
		//			while(pressed_button == A);
		//			HiWave();
		//		}

		if (pressed_button == B){
			//		HAL_UART_Transmit_IT(&huart1, data, strlen(data));
			while(pressed_button == B);
			if(mode == TROT){
				mode = WALK;
				ticks=0;
				step_height = 0.038;
				step_length = 0.00032;
			}else if (mode == WALK){
				mode = TROT;
				step_height = 0.042;
				ticks=0;
				step_length = 0.00028;
			}

			sprintf(data,"%d %d %s 0\r\n",(int)(-mpu6050.roll), (int)(mpu6050.pitch), mode? "Trot":"Walk");
			HAL_UART_Transmit_IT(&huart1, data, strlen(data));
		}

		if(ticks == 0 && ticks == 2 && ticks == 3 && ticks == 5 && mode == TROT){
			HAL_Delay(120);
		}else if (mode == TROT){
			HAL_Delay(175);
		}else{
			HAL_Delay(250);
		}

		// Initilization for Servo Motors
		//				FL_angles[0] = 1.57;
		//				FL_angles[1] = 1.57;
		//				FL_angles[2] = 1.57;

		//				FR_angles[0] = 1.57;
		//				FR_angles[1] = 1.57;
		//				FR_angles[2] = 1.57;
		//
		//				BL_angles[0] = 1.57;
		//				BL_angles[1] = 1.57;
		//				BL_angles[2] = 1.57;
		//
		//				BR_angles[0] = 1.57;
		//				BR_angles[1] = 1.57;
		//				BR_angles[2] = 1.57;

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
	FL_offsets[0] = 0.21;
	FL_offsets[1] = -0.10;
	FL_offsets[2] = -0.26;
	FR_offsets[0] = -0.09;
	FR_offsets[1] = 0.02;
	FR_offsets[2] = -0.26;
	BL_offsets[0] = 0.09;
	BL_offsets[1] = -0.08;
	BL_offsets[2] = -0.04;
	BR_offsets[0] = -0.01;
	BR_offsets[1] = 0.15;
	BR_offsets[2] = 0.39;
	ServoDriverInit(&pca9865, &hi2c1, SERVO_DRIVER_ADDRESS);
	HAL_Delay(300);
	MPUInit(&mpu6050, &hi2c2, MPU6050_DataRate_2KHz, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s, 0.005, 0.5);
	MPUSetOffsets(&mpu6050, -944, -600, -590, 1130, 16, 923);
	SPIDInit(&pid_pitch, &pitch_error, &pitch_output, 0.01, 1.0, 0.3, 0.21, 0.02, 0.004, 1.0/30.0, 0.2);
	SPIDInit(&pid_roll, &roll_error, &roll_output, 0.01, 1.0, 0.3, 0.21, 0.02, 0.004, 1.0/30.0, 0.2);
	//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	/* Infinite loop */
	for(;;)
	{
		//		osSemaphoreAcquire(sendSemaphoreHandle, osWaitForever);
		MPUReqAccGyro(&mpu6050);
		CompPitchRoll(&mpu6050);
		pitch_error = req_body_rotation[0] - (-mpu6050.roll + 0.75);  // Increase to move back
		roll_error = req_body_rotation[1] - (mpu6050.pitch - 11.0);  // decrease to tilt right
		SPIDLoop(&pid_pitch);
		SPIDLoop(&pid_roll);
		pitch_sum+=pitch_output;
		if (pitch_sum > 0.3) pitch_sum = 0.3;
		if (pitch_sum < -0.3) pitch_sum = -0.3;
		roll_sum+=roll_output;
		if (roll_sum > 0.3) roll_sum = 0.3;
		if (roll_sum < -0.3) roll_sum = -0.3;

		body_rotation[0]= pitch_sum;
		body_rotation[1]= roll_sum;

		//		sprintf(data," Pitch: %d  Roll: %d \r\n", (int)(-mpu6050.roll), (int)(mpu6050.pitch));
		//				sprintf(data," %d %d \r\n",(int)J1y, (int)J1x);
		//		HAL_UART_Transmit_IT(&huart1, data, strlen(data));

		inverse_kinematics_all(FL_position,FR_position,BL_position, BR_position, FL_body_angles, FR_body_angles, BL_body_angles, BR_body_angles);
		load_angles();
		ServoDriverSetOnOff_Multi(&pca9865,0,12,pulses);
		//		ServoDriverSetOnOff(&pca9865,1,0,pulses[1]);
		//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(10);
		//		osDelay(1000);
	}
	/* USER CODE END CalcFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	if(rx != '\n'){
		data[rx_index] = rx;
		rx_index +=1;
	}else{
		data[rx_index] = rx;
		rx_index = 0;

		switch(data[0]){
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
		case 'L' :
			pressed_button = L1;
			break;
		case 'R' :
			pressed_button = R1;
			break;
		case 'S' :
			if(data[1] == 'T'){
				pressed_button = START;
			}else if(data[1] == '1'){

				uint8_t joy_index = 2;
				memset(num_conv,'_',6);
				while( data[joy_index] != '\n'){
					num_conv[joy_index] = data[joy_index];
					joy_index+=1;
				}
				slider_speed = (uint8_t)(atoi(num_conv));

			}else if(data[1] == '2'){

				uint8_t joy_index = 2;
				memset(num_conv,'_',6);
				while( data[joy_index] != '\n'){
					num_conv[joy_index] = data[joy_index];
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
			if(data[1] == '1'){
				uint8_t joy_index = 2;
				uint8_t second_index = 0;
				memset(num_conv,'_',6);

				while( data[joy_index] != ' '){
					num_conv[second_index] = data[joy_index];
					joy_index+=1;
					second_index+=1;
				}

				J1x = (atoi(num_conv));
				//				if(J1x > 100) J1x = 100;
				//				if(J1x < -100) J1x = -100;

				memset(num_conv,'_',6);

				second_index = 0;
				joy_index+=1;

				while( data[joy_index] != '\n'){
					num_conv[second_index] = data[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J1y = (atoi(num_conv));
				//				if(J1y > 100) J1y = 100;
				//				if(J1y < -100) J1y = -100;

			}else if(data[1] == '2'){
				uint8_t joy_index = 2;
				uint8_t second_index = 0;
				memset(num_conv,'_',6);

				while( data[joy_index] != ' '){
					num_conv[second_index] = data[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J2x = (atoi(num_conv));
				//				if(J2x > 100) J2x = 100;
				//				if(J2x < -100) J2x = -100;

				memset(num_conv,'_',6);
				second_index = 0;
				joy_index+=1;

				while( data[joy_index] != '\n'){
					num_conv[second_index] = data[joy_index];
					joy_index+=1;
					second_index+=1;
				}
				J2y = (atoi(num_conv));
				//				if(J2y > 100) J2y = 100;
				//				if(J2y < -100) J2y = -100;
			}
			break;
		}
		memset(data,'0',20);
	}

	HAL_UART_Receive_IT(&huart1, &rx, 1);

}

void inverse_leg_kinematics(float position[], float angles[], uint8_t leg_index, float rotation[]){

	if(position[0] == 0.0) position[0] = -0.00001;
	if(position[1] == 0.0) position[1] = -0.00001;


	float pitch_offset = sin(rotation[0]) * (body_length/2);
	float x_pitch_offset = ((position[2] - pitch_offset) * tan(rotation[0])  * 0.2);

	float roll_offset = sin(rotation[1]) * (body_width/2);
	float y_roll_offset = ((position[2] - roll_offset) * tan(rotation[1]) * 0.2);
	float current_yaw,new_width, new_length, H_leg, new_yaw,new_height,new_x,new_y,H1y,H2y,phi, phi2,theta,Hx,trident ;

	switch(leg_index){
	case 1:
		new_height = position[2] - pitch_offset + roll_offset;
		new_y = -position[1] + y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset;
		H_leg = hypot(-body_width/2.0 + new_y,body_length/2.0 + new_x);
		current_yaw = atan2(body_length/2.0 + new_x, -body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length - body_length/2.0;
		new_y = new_width + body_width/2.0 - balance_offset;
		break;
	case 2:
		new_height = position[2] - pitch_offset - roll_offset;
		new_y = position[1] - y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset;
		H_leg = hypot(body_width/2.0 + new_y,body_length/2.0 + new_x);
		current_yaw = atan2(body_length/2.0 + new_x, body_width/2.0 + new_y);
		new_yaw = current_yaw - rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length - body_length/2.0;
		new_y = new_width - body_width/2.0 - balance_offset;
		break;
	case 3:
		new_height = position[2] + pitch_offset + roll_offset;
		new_y = -position[1] + y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset;
		H_leg = hypot(-body_width/2.0 + new_y,-body_length/2.0 + new_x);
		current_yaw = atan2(-body_length/2.0 + new_x, -body_width/2.0 + new_y);
		new_yaw = current_yaw + rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length + body_length/2.0  - hind_lef_offest;
		new_y = new_width + body_width/2.0 - balance_offset;
		break;
	case 4:
		new_height = position[2] + pitch_offset - roll_offset;
		new_y = position[1] - y_roll_offset + Y_offset;
		new_x = position[0] - x_pitch_offset - X_offset;
		H_leg = hypot(body_width/2.0 + new_y,-body_length/2.0 + new_x);
		current_yaw = atan2(-body_length/2.0 + new_x, body_width/2.0 + new_y);
		new_yaw = current_yaw - rotation[2];
		new_width = cos(new_yaw) * H_leg;
		new_length = sin(new_yaw) * H_leg;
		new_x = new_length + body_length/2.0 - hind_lef_offest;
		new_y = new_width - body_width/2.0 - balance_offset;
		break;
	}

	H1y = hypot(new_y,new_height);
	H2y = sqrt(pow(H1y,2)- pow(Y_offset,2));
	phi = atan2(new_y,new_height);
	phi2 = acos(Y_offset/H2y);
	angles[0] = phi  + phi2;

	theta = atan2(new_x,H2y);
	Hx = hypot(new_x,H2y);
	trident =  acos( (pow(Hx,2) + pow(upper_leg_len,2) - pow(lower_leg_len,2)) / (2 * Hx * upper_leg_len) );
	angles[1] = trident - theta + 1.5707;
	angles[2] = acos( (pow(lower_leg_len,2) + pow(upper_leg_len,2) - pow(Hx,2)) / (2 * lower_leg_len * upper_leg_len) );
	//	adjust_servo_angles(angles,leg_index);

}


void inverse_kinematics_all(float FL_position[], float FR_position[], float BL_position[], float BR_position[], float FL_rotation[], float FR_rotation[], float BL_rotation[], float BR_rotation[]){

	smoothed_FL_position[0] = (FL_position[0] * smoothing_var) + (prev_FL_position[0] * (1.0-smoothing_var));
	prev_FL_position[0] = smoothed_FL_position[0];

	smoothed_FL_position[1] = FL_position[1] * smoothing_var + prev_FL_position[1] * (1.0-smoothing_var);
	prev_FL_position[1] = smoothed_FL_position[1];

	smoothed_FL_position[2] = FL_position[2] * smoothing_var + prev_FL_position[2] * (1.0-smoothing_var);
	prev_FL_position[2] = smoothed_FL_position[2];

	smoothed_FR_position[0] = FR_position[0] * smoothing_var + prev_FR_position[0] * (1.0-smoothing_var);
	prev_FR_position[0] = smoothed_FR_position[0];

	smoothed_FR_position[1] = FR_position[1] * smoothing_var + prev_FR_position[1] * (1.0-smoothing_var);
	prev_FR_position[1] = smoothed_FR_position[1];

	smoothed_FR_position[2] = FR_position[2] * smoothing_var + prev_FR_position[2] * (1.0-smoothing_var);
	prev_FR_position[2] = smoothed_FR_position[2];

	smoothed_BL_position[0] = BL_position[0] * smoothing_var + prev_BL_position[0] * (1.0-smoothing_var);
	prev_BL_position[0] = smoothed_BL_position[0];

	smoothed_BL_position[1] = BL_position[1] * smoothing_var + prev_BL_position[1] * (1.0-smoothing_var);
	prev_BL_position[1] = smoothed_BL_position[1];

	smoothed_BL_position[2] = BL_position[2] * smoothing_var + prev_BL_position[2] * (1.0-smoothing_var);
	prev_BL_position[2] = smoothed_BL_position[2];

	smoothed_BR_position[0] = BR_position[0] * smoothing_var + prev_BR_position[0] * (1.0-smoothing_var);
	prev_BR_position[0] = smoothed_BR_position[0];

	smoothed_BR_position[1] = BR_position[1] * smoothing_var + prev_BR_position[1] * (1.0-smoothing_var);
	prev_BR_position[1] = smoothed_BR_position[1];

	smoothed_BR_position[2] = BR_position[2] * smoothing_var + prev_BR_position[2] * (1.0-smoothing_var);
	prev_BR_position[2] = smoothed_BR_position[2];

	inverse_leg_kinematics(smoothed_FL_position, FL_angles, 1, FL_rotation);
	inverse_leg_kinematics(smoothed_FR_position, FR_angles, 2, FR_rotation);
	inverse_leg_kinematics(smoothed_BL_position, BL_angles, 3, BL_rotation);
	inverse_leg_kinematics(smoothed_BR_position, BR_angles, 4, BR_rotation);

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

	pulses[0] = (650 - angle_to_pulse(FL_angles[0] + FL_offsets[0])); //650

	pulses[1] = (650 - angle_to_pulse(FL_angles[1] + FL_offsets[1]));

	pulses[2] = angle_to_pulse(FL_angles[2] + FL_offsets[2]);

	pulses[3] = angle_to_pulse(FR_angles[0] + FR_offsets[0]);

	pulses[4] = angle_to_pulse(FR_angles[1] + FR_offsets[1]) ;

	pulses[5] = (650 - angle_to_pulse(FR_angles[2] + FR_offsets[2]));

	pulses[6] = angle_to_pulse(BL_angles[0] + BL_offsets[0]);

	pulses[7] = (650 - angle_to_pulse(BL_angles[1] + BL_offsets[1]));

	pulses[8] = angle_to_pulse(BL_angles[2] + BL_offsets[2]);

	pulses[9] = (650 - angle_to_pulse(BR_angles[0] + BR_offsets[0]));

	pulses[10] = angle_to_pulse(BR_angles[1] + BR_offsets[1]) ;

	pulses[11] = (650 - angle_to_pulse(BR_angles[2] + BR_offsets[2]));


}

void Stand(){
	load_leg_position(FL_position,0.0,0.0,body_height, FL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
	load_leg_position(FR_position,0.0,0.0,body_height, FR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
	load_leg_position(BL_position,0.0,0.0,body_height, BL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
	load_leg_position(BR_position,0.0,0.0,body_height, BR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
}

void Gait_controller (uint8_t ticks, float x_setpoint, float y_setpoint){

	if(mode == WALK){

		switch(ticks){

		case 0:
			load_leg_position(FL_position,x_setpoint - x_translation,y_setpoint + y_translation  ,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint + y_translation ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint + y_translation ,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,-x_setpoint - x_translation,-y_setpoint + y_translation ,body_height - step_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 1:
			load_leg_position(FL_position,0.0 - x_translation,0.0 + y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint + y_translation  ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,0.0 - x_translation,0.0 + y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint + y_translation  ,body_height - step_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 2:
			load_leg_position(FL_position,0.0 - x_translation,0.0 + y_translation ,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint + y_translation ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,0.0 - x_translation,0.0 + y_translation ,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint + y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 3:
			load_leg_position(FL_position,0.0 - x_translation,0.0 + y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint + y_translation ,body_height - step_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,0.0 - x_translation,0.0 + y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint + y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 4:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint + y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,x_setpoint - x_translation,y_setpoint  + y_translation,body_height - step_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,-x_setpoint - x_translation,-y_setpoint + y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint  + y_translation,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 5:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,x_setpoint - x_translation,y_setpoint  - y_translation,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint  - y_translation,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 6:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,x_setpoint - x_translation,y_setpoint  - y_translation,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height - step_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,x_setpoint - x_translation,y_setpoint  - y_translation,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 7:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,0.0 - x_translation,0.0  - y_translation,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint- y_translation ,body_height - step_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,0.0 - x_translation,0.0 - y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 8:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint- y_translation ,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,0.0 - x_translation,0.0 - y_translation ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint - y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,0.0 - x_translation,0.0 - y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;

		case 9:
			load_leg_position(FL_position,-x_setpoint - x_translation,-y_setpoint - y_translation,body_height - step_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,0.0 - x_translation,0.0 - y_translation ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint- y_translation ,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,0.0 - x_translation,0.0 - y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 10:
			load_leg_position(FL_position,x_setpoint - x_translation,y_setpoint- y_translation ,body_height - step_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint - y_translation ,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint - y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,-x_setpoint- x_translation ,-y_setpoint - y_translation ,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		case 11:
			load_leg_position(FL_position,x_setpoint - x_translation,y_setpoint + y_translation,body_height, FL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(FR_position,-x_setpoint - x_translation,-y_setpoint  + y_translation,body_height, FR_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BL_position,x_setpoint - x_translation,y_setpoint + y_translation,body_height, BL_body_angles, body_rotation[0], body_rotation[1], 0);
			load_leg_position(BR_position,-x_setpoint - x_translation,-y_setpoint  + y_translation,body_height, BR_body_angles, body_rotation[0], body_rotation[1], 0);
			break;
		}

	}else if(mode == TROT){
		switch(ticks){

		case 0:
			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height-step_height, FL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height, FR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height, BL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height-step_height, BR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			break;
		case 1:
			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height, FL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height, FR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height, BL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height, BR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			break;
		case 2:
			load_leg_position(FL_position,x_setpoint,y_setpoint,body_height, FL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(FR_position,-x_setpoint,-y_setpoint,body_height - step_height, FR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BL_position,-x_setpoint,-y_setpoint,body_height - step_height, BL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(BR_position,x_setpoint,y_setpoint,body_height, BR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			break;
		case 3:
			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height, FL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height - step_height, FR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height - step_height, BL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height, BR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			break;
		case 4:
			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height, FL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height, FR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height, BL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height, BR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			break;
		case 5:
			load_leg_position(FL_position,-x_setpoint,-y_setpoint,body_height - step_height, FL_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			load_leg_position(FR_position,x_setpoint,y_setpoint,body_height, FR_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BL_position,x_setpoint,y_setpoint,body_height, BL_body_angles, body_rotation[0], body_rotation[1], body_rotation[2]);
			load_leg_position(BR_position,-x_setpoint,-y_setpoint,body_height - step_height, BR_body_angles, body_rotation[0], body_rotation[1], -body_rotation[2]);
			break;
		}

	}
	//	else if (mode == GALLOP){
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
	//	}

}

int angle_to_pulse(float angle){
	int pulse;

	pulse = (uint16_t)(angle * 90.7549) + 115;

	if (pulse > 535) pulse = 535;
	if (pulse < 115) pulse = 115;

	return pulse;
}
/* USER CODE END Application */

void Rise(){
//
//	float position = 0.02;
//	for ( int j = 0; j < 100; j++){
//		position -= 0.0003;
//		FL_position[1] = position;
//		FR_position[1] = position;
//		BL_position[1] = position;
//		BR_position[1] = position;
//		HAL_Delay(1);
//	}

	float position = 0.04;
	for ( int j = 0; j < 100; j++){
		position += 0.0013;
		FL_position[2] = position;
		FR_position[2] = position;
		BL_position[2] = position;
		BR_position[2] = position;
		HAL_Delay(10);
	}
}

//void HiWave(){
//
//
//	HAL_Delay(200);
//
//	body_rotation[0] = -0.35;
//	FL_position[2] = 0.15;
//
//	HAL_Delay(500);
//
//	FR_position[2] = 0.06;
//	FR_position[0] = 0.12;
//
//
//	for (int i = 0; i < 4; i++){
//		HAL_Delay(500);
//
//		FR_position[1] = 0.01;
//
//		HAL_Delay(500);
//
//		FR_position[1] = -0.01;
//	}
//
//}

/* USER CODE END Application */

