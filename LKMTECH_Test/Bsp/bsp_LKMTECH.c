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
  *  V1.0.0     Nove-28-2024   chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "bsp_LKMTECH.h"
#include "can.h"

CAN_TxHeaderTypeDef lkmtech_tx_message;
static uint8_t lkmtech_can_send_data[8];
static uint8_t lkmtech_can_receive_data[8];

void LKMTECH_motor_cantx_communication(uint8_t id, uint8_t* data)
{
  uint32_t send_mail_box;
  
  lkmtech_tx_message.StdId = 0x140 + id;
  lkmtech_tx_message.IDE = CAN_ID_STD;
  lkmtech_tx_message.RTR = CAN_RTR_DATA;
  lkmtech_tx_message.DLC = 0x08;

  HAL_CAN_AddTxMessage(&hcan1, &lkmtech_tx_message, data, &send_mail_box);
}

void LKMTECH_read_motor_state(uint8_t id, uint8_t state_type)
{
  for(int i = 0; i<8 ; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  if(state_type == 0)
  {
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE1_CMD;
  }
  else if(state_type == 1)
  {
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE2_CMD;
  }
  else
  {
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE3_CMD;
  }

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}

void LKMTECH_clear_motor_error(uint8_t id);


void LKMTECH_motor_disable(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_DISABLE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_enable(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_ENABLE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}



void LKMTECH_motor_free(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_FREE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_brake(uint8_t id, uint8_t brakeControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_BRAKE_CMD;
  lkmtech_can_send_data[1] = brakeControl;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_openloop_control_mode(uint8_t id, int16_t powerControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_OPENLOOP_CONTROL_CMD;
  lkmtech_can_send_data[4] = *(uint8_t*)(&powerControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&powerControl)+1);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_torque_control_mode(uint8_t id, int16_t iqControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_TORQUE_CONTROL_CMD;
  lkmtech_can_send_data[4] = *(uint8_t*)(&iqControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&iqControl)+1);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_speed_control_mode1(uint8_t id, int32_t speedControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_SPEED1_CONTROL_CMD;
  lkmtech_can_send_data[4] = *(uint8_t*)(&speedControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&speedControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&speedControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&speedControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_speed_control_mode2(uint8_t id, int32_t speedControl, int16_t iqControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_SPEED2_CONTROL_CMD;
  lkmtech_can_send_data[2] = *(uint8_t*)(&iqControl);
  lkmtech_can_send_data[3] = *((uint8_t*)(&iqControl)+1);  
  lkmtech_can_send_data[4] = *(uint8_t*)(&speedControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&speedControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&speedControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&speedControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_multiposition_control_mode1(uint8_t id, int32_t angleControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_MULTIPOSITION1_CONTROL_CMD;
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_multiposition_control_mode2(uint8_t id, int32_t angleControl, uint16_t maxSpeed)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_MULTIPOSITION2_CONTROL_CMD;
  lkmtech_can_send_data[2] = *(uint8_t*)(&maxSpeed);
  lkmtech_can_send_data[3] = *((uint8_t*)(&maxSpeed)+1);  
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_singleposition_control_mode1(uint8_t id, uint32_t angleControl, uint8_t spinDirection)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_SINGLEPOSITION1_CONTROL_CMD;
  lkmtech_can_send_data[1] = spinDirection;
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_singleposition_control_mode2(uint8_t id, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_SINGLEPOSITION2_CONTROL_CMD;
  lkmtech_can_send_data[1] = spinDirection;
  lkmtech_can_send_data[2] = *(uint8_t*)(&maxSpeed);
  lkmtech_can_send_data[3] = *((uint8_t*)(&maxSpeed)+1);  
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleControl);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleControl)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleControl)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleControl)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


void LKMTECH_motor_incrementposition_control_mode1(uint8_t id, int32_t angleIncrement)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_INCREMENTALPOSITION1_CONTROL_CMD;
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleIncrement);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleIncrement)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleIncrement)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleIncrement)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}

void LKMTECH_motor_incrementposition_control_mode2(uint8_t id, int32_t angleIncrement, uint32_t maxSpeed)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_INCREMENTALPOSITION2_CONTROL_CMD;
  lkmtech_can_send_data[2] = *(uint8_t*)(&maxSpeed);
  lkmtech_can_send_data[3]= *((uint8_t*)(&maxSpeed)+1);
  lkmtech_can_send_data[4] = *(uint8_t*)(&angleIncrement);
  lkmtech_can_send_data[5] = *((uint8_t*)(&angleIncrement)+1);
  lkmtech_can_send_data[6] = *((uint8_t*)(&angleIncrement)+2);
  lkmtech_can_send_data[7] = *((uint8_t*)(&angleIncrement)+3);

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


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
