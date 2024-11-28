/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       bsp_LKMTECH.c/h
  * @brief     
  *             这里是瓴控电机can驱动文件，包含can通讯收发，电流模式，位置模式
  *             运控模式，速度模式控制API，以及电机参数读取和写入API。
  *             
  * @note      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nove-28-2024   chenjiangnan    1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#ifndef __BSP_LKMTECH_H
#define __BSP_LKMTECH_H

#include "main.h"

#define LKMTECH_READ_MOTOR_STATE1_CMD 0x9A
#define LKMTECH_CLEAR_MOTOR_ERROR_CMD 0x9B
#define LKMTECH_READ_MOTOR_STATE2_CMD 0x9C
#define LKMTECH_READ_MOTOR_STATE3_CMD 0x9D
#define LKMTECH_MOTOR_DISABLE_CMD 0x80
#define LKMTECH_MOTOR_ENABLE_CMD 0x88
#define LKMTECH_MOTOR_FREE_CMD 0x81
#define LKMTECH_MOTOR_BRAKE_CMD 0x8C
#define LKMTECH_MOTOR_OPENLOOP_CONTROL_CMD 0xA0
#define LKMTECH_MOTOR_TORQUE_CONTROL_CMD 0xA1
#define LKMTECH_MOTOR_SPEED1_CONTROL_CMD 0xA2
#define LKMTECH_MOTOR_SPEED2_CONTROL_CMD 0xAD
#define LKMTECH_MOTOR_MULTIPOSITION1_CONTROL_CMD 0xA3
#define LKMTECH_MOTOR_MULTIPOSITION2_CONTROL_CMD 0xA4
#define LKMTECH_MOTOR_SINGLEPOSITION1_CONTROL_CMD 0xA5
#define LKMTECH_MOTOR_SINGLEPOSITION2_CONTROL_CMD 0xA6
#define LKMTECH_MOTOR_INCREMENTALPOSITION1_CONTROL_CMD 0xA6
#define LKMTECH_MOTOR_INCREMENTALPOSITION2_CONTROL_CMD 0xA8
#define LKMTECH_MOTOR_READ_CONTROL_CONFIG_CMD 0xB0
#define LKMTECH_MOTOR_SET_CONTROL_CONFIG_CMD 0xB1
#define LKMTECH_MOTOR_WRITE_SETENCODE_TO_ZERO_CMD 0x91
#define LKMTECH_MOTOR_WRITE_RELENCODE_TO_ZERO_CMD 0x19
#define LKMTECH_MOTOR_READ_MUTLIPOSITION_CMD 0x92
#define LKMTECH_MOTOR_READ_SINGLEPOSITION_CMD 0x94
#define LKMTECH_MOTOR_SET_MULTIPOSITION_TO_RAM_CMD 0x95


void LKMTECH_motor_cantx_communication(uint8_t id, uint8_t* data);

void LKMTECH_read_motor_state(uint8_t id, uint8_t state_type);


void LKMTECH_clear_motor_error(uint8_t id);


void LKMTECH_motor_disable(uint8_t id);


void LKMTECH_motor_enable(uint8_t id);


void LKMTECH_motor_free(uint8_t id);



void LKMTECH_motor_brake(uint8_t id, uint8_t brakeControl);



void LKMTECH_motor_openloop_control_mode(uint8_t id, int16_t powerControl);



void LKMTECH_motor_torque_control_mode(uint8_t id, int16_t iqControl);



void LKMTECH_motor_speed_control_mode1(uint8_t id, int32_t speedControl);



void LKMTECH_motor_speed_control_mode2(uint8_t id, int32_t speedControl, int16_t iqControl);



void LKMTECH_motor_multiposition_control_mode1(uint8_t id, int32_t angleControl);



void LKMTECH_motor_multiposition_control_mode2(uint8_t id, int32_t angleControl, uint16_t maxSpeed);



void LKMTECH_motor_singleposition_control_mode1(uint8_t id, uint32_t angleControl, uint8_t spinDirection);



void LKMTECH_motor_singleposition_control_mode2(uint8_t id, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed);



void LKMTECH_motor_incrementposition_control_mode1(uint8_t id, int32_t angleIncrement);


void LKMTECH_motor_incrementposition_control_mode2(uint8_t id, int32_t angleIncrement, uint32_t maxSpeed);



// void LKMTECH_read_motor_control_config()
// {
//   for(int i = 0; i<8; i++)
//   {
//     lkmtech_can_send_data[i] = 0;
//   }

//   lkmtech_can_send_data[0] = LKMTECH_MOTOR_READ_CONTROL_CONFIG_CMD;
//   lkmtech_can_send_data[2] = *(uint8_t*)(&maxSpeed);
//   lkmtech_can_send_data[3]= *((uint8_t*)(&maxSpeed)+1);
//   lkmtech_can_send_data[4] = *(uint8_t*)(&angleIncrement);
//   lkmtech_can_send_data[5] = *((uint8_t*)(&angleIncrement)+1);
//   lkmtech_can_send_data[6] = *((uint8_t*)(&angleIncrement)+2);
//   lkmtech_can_send_data[7] = *((uint8_t*)(&angleIncrement)+3);

//   LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
// }


void LKMTECH_set_motor_control_config(void);
void LkMTECH_read_motor_encode(void);
void LKMTECH_write_motor_setencode_to_zero(void);
void LKMTECH_write_motor_relencode_to_zero(void);
void LKMTECH_read_motor_multiposition(void);
void LKMTECH_read_motor_singleposition(void);
// void LKMTECH_set

#endif
