/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PCA9865.h"
#include "MPU6050.h"
#include "spid.h"
#include "i2c.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"
#include "adc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define Y_offset 0.00  //0.02
#define X_offset -0.001  // -0.022
#define upper_leg_len 0.13
#define lower_leg_len 0.115
#define body_width 0.1
#define body_length 0.15
#define hind_leg_offest 0.035 //0.055
#define smoothing_var 1.0
#define balance_offset 0.000
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
//extern uint16_t pulses[12];
//extern uint16_t prev_pulses[12];

//offsets
//extern float FL_offsets[3];
//extern float FR_offsets[3];
//extern float BL_offsets[3];
//extern float BR_offsets[3];
////position
//extern float FL_position[3];
//extern float FR_position[3];
//extern float BL_position[3];
//extern float BR_position[3];
////angles
//extern float FL_angles[3];
//extern float FR_angles[3];
//extern float BL_angles[3];
//extern float BR_angles[3];
//
//extern float body_rotation[3];
//extern float step_height;
//extern float body_height;
//extern uint8_t ticks;
//
//extern float pitch_error, pitch_output, pitch_sum;
//extern float roll_error, roll_output, roll_sum;
//
//extern char data[30];
//extern char num_conv[6];
//extern char rx;
//extern uint8_t rx_index;
//extern uint8_t pressed_button;
//extern uint8_t slider_speed, slider_angle;
//extern int J1x , J1y , J2x, J2y;
//extern uint8_t mode;

void inverse_leg_kinematics(float position[], float angles[], uint8_t leg_index, float rotation[]);
void adjust_servo_angles(float angles[], uint8_t leg_index);
void inverse_kinematics_all(void);
void load_angles();
void Gait_controller (uint32_t ticks, float x_setpoint, float y_setpoint, float turn_setpoint);
int angle_to_pulse(float angle);
void leg_cycle(float positions[], uint8_t leg, uint8_t mode, uint32_t ticks, float x_setpoint, float y_setpoint, float angles[], float p, float r, float yaw);
void Stand();
float generate_linear_xy_trajectory(float min, float max, float interpolation);
float generate_cubic_z_trajectory(float t);
float generate_step_z_trajectory(float t, float max_height);
float generate_smooth_xy_trajectory(float min, float max, float t);
float generate_smooth_z_trajectory(float t, float height);
void Rise();
void HiWave();
void Scratch();
void Step_Forward();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
