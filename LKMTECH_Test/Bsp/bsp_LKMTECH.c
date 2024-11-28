/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       bsp_LKMTECH.c/h
  * @brief     
  *             这里是瓴控电机can驱动文件，包含can通讯收发，电流模式，位置模式
  *             运控模式，速度模式控制API，以及电机参数读取和写入API。
  *             
  * @note      1.同一总线上共可以挂载多达32（视总线负载情况而定）个驱动，为了防止总线冲突，
  *             每个驱动需要设置不同的ID
  *            2.主控向总线发送单电机命令，对应ID的电机在收到命令后执行，并在一段时间后
  *            （0.25ms内）向主控发送回复。
  * 
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


/**
  * @name           LKMTECH_motor_init
  * @brief          瓴控电机初始化函数
  * @param[in]      device:瓴控电机句柄
  * @param[in]      id:设备CAN ID，can报文标准ID 0x140 + 设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_init(LKMTECH_DataHandleTypeDef *device, uint8_t id)
{
  /*初始化id*/
  device->id = id;
  /*初始化base data*/
  device->baseData.speed = 0;
  device->baseData.encoder = 0;
  device->baseData.iq = 0;
  device->baseData.current = 0;
  device->baseData.voltage = 0;
  device->baseData.temperature = 0;
  /*初始化control data*/
  device->controlData.mode = 0;
  device->controlData.ctlFrequency = 0;
  device->controlData.revFrequency = 0;
  /*初始化*/
  device->motorState = 0;
  device->errorState = 0;
}


/**
  * @name           LKMTECH_motor_canrx_communication
  * @brief          瓴控电机can数据包解析函数
  * @param[in]      device:瓴控电机句柄
  * @param[in]      canrx_data:can接收报文
  * @retval         none
  */
void LKMTECH_motor_canrx_communication(LKMTECH_DataHandleTypeDef *device, uint8_t* canrx_data)
{
  if(device || canrx_data)
  {
    return;
  }

  if(canrx_data[0] == 0x9A)
  {
    device->baseData.temperature = canrx_data[1];
    device->baseData.voltage = canrx_data[2] || canrx_data[3]<<8;
    device->baseData.current = canrx_data[4] || canrx_data[5]<<8;
    device->motorState = canrx_data[6]; 
    device->errorState = canrx_data[7];
  }
  else if(canrx_data[0] == 0x9D){
    device->baseData.temperature = canrx_data[1];
    device->baseData.iA = canrx_data[2] || canrx_data[3]<<8;
    device->baseData.iB = canrx_data[4] || canrx_data[5]<<8;
    device->baseData.iC = canrx_data[6] || canrx_data[7]<<8;
  }
  else{
    device->baseData.temperature = canrx_data[1];
    device->baseData.iq = canrx_data[2] || canrx_data[3]<<8;
    device->baseData.speed = canrx_data[4] || canrx_data[5]<<8;
    device->baseData.encoder = canrx_data[6] || canrx_data[7]<<8;
  }
}

/**
  * @name           LKMTECH_motor_cantx_communication
  * @brief          电机 CAN报文 发送函数 
  * @note           电机通信为CAN 2.0通信接口，波特率1Mbps，采用标准帧格式
  * @param[in]      id:设备CAN ID，can报文标准ID 0x140 + 设备id(1~32)
  * @param[in]      data:数据指针，can报文数据位 Byte0~Byte7
  * @retval         none
  */
void LKMTECH_motor_cantx_communication(uint8_t id, uint8_t* data)
{
  uint32_t send_mail_box;

  lkmtech_tx_message.StdId = 0x140 + id;
  lkmtech_tx_message.IDE = CAN_ID_STD;
  lkmtech_tx_message.RTR = CAN_RTR_DATA;
  lkmtech_tx_message.DLC = 0x08;

  HAL_CAN_AddTxMessage(&hcan1, &lkmtech_tx_message, data, &send_mail_box);
}


/**
  * @name           LKMTECH_read_motor_state
  * @brief          读取电机状态
  * @note           电机状态信息分为3类（state1, state2, state3）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      state_type:状态信息
  * @retval         none
  */
void LKMTECH_read_motor_state(uint8_t id, uint8_t state_type)
{
  for(int i = 0; i<8 ; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  if(state_type == 0)
  {
    /*该命令读取当前电机的温度、电压和错误状态标志*/
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE1_CMD;
  }
  else if(state_type == 1)
  {
    /*该命令读取当前电机的温度、电机转矩电流（MF、MG）/电机输出功率（MS）、转速、编码器位*/
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE2_CMD;
  }
  else
  {
    /*由于MS电机没有相电流采样，该命令在MS电机上无作用。
    该命令读取当前电机的温度和3相电流数据 */ 
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE3_CMD;
  }

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}



/**
  * @name           LKMTECH_clear_motor_error
  * @brief          清除电机错误标志命令
  * @note           电机在收到命令后回复主机。回复数据和读取电机状态1和错误标志命令相同（仅命令字节DATA[0]不同，这里为0x9B）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_clear_motor_error(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_CLEAR_MOTOR_ERROR_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


/**
  * @name           LKMTECH_motor_disable
  * @brief          电机失能
  * @note           将电机从开启状态（上电后默认状态）切换到关闭状态，
  *                 清除电机转动圈数及之前接收的控制指令，LED由常亮转为慢闪。
  *                 此时电机仍然可以回复控制命令，但不会执行动作。
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_disable(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_DISABLE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


/**
  * @name           LKMTECH_motor_enable
  * @brief          电机使能
  * @note           将电机从关闭状态切换到开启状态，LED由慢闪转为常亮。
  *                 此时再发送控制指令即可控制电机动作。
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_enable(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_ENABLE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


/**
  * @name           LKMTECH_motor_free
  * @brief          停止电机
  * @note           停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_free(uint8_t id)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_FREE_CMD;

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


/**
  * @name           LKMTECH_motor_brake
  * @brief          控制抱闸开合
  * @note           控制抱闸器的开合，或者读取当前抱闸器的状态。 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      brakeControl:0x00：抱闸器断电，刹车启动 
  *                              0x01：抱闸器通电，刹车释放
  *                              0x10：读取抱闸器状态 
  * @retval         none
  */
void LKMTECH_motor_brake(uint8_t id, uint8_t brakeControl)
{
  for(int i = 0; i<8; i++)
  {
    lkmtech_can_send_data[i] = 0;
  }

  lkmtech_can_send_data[0] = LKMTECH_MOTOR_BRAKE_CMD;
  if(brakeControl>=0 && brakeControl<=2)
  {
    lkmtech_can_send_data[1] = brakeControl;
  }

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}


/**
  * @name           LKMTECH_motor_openloop_control_mode
  * @brief          开环控制命令
  * @note           主机发送该命令以控制输出到电机的开环电压，控制值powerControl
  *                 为int16_t类型，数值范围-850~850，（电机电流和扭矩因电机而异）
  *                （该命令仅在MS电机上实现，其他电机无效）
  * 
  *                 1.该命令中的控制值powerControl不受上位机中的Max Power值限制 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      powerControl:[-850,850]
=
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_torque_control_mode
  * @brief          转矩闭环控制命
  * @note           主机发送该命令以控制电机的转矩电流输出，控制值iqControl为int16_t类型，数值范围-2048~ 2048，
  *                 对应MF电机实际转矩电流范围-16.5A~16.5A，对应MG电机实际转矩电流范围-33A~33A，母线电流和电机
  *                 的实际扭矩因不同电机而异。 
  *                （该命令仅在MF、MH、MG电机上实现）
  *               
  *                 1. 该命令中的控制值iqControl不受上位机中的Max Torque Current值限制
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同这里为0xA1）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      iqControl:[-2048, 2048] -> [-16.5A 16.5A]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_speed_control_mode1
  * @brief          速度闭环控制命令1
  * @note           主机发送该命令以控制电机的速度， 控制值 speedControl 为 int32_t 类型，对应实际转速为0.01dps/LSB。
  * 
  *                 1.该命令下电机的speedControl由上位机中的Max Speed值限制
  *                 2.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。
  *                 3.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA2）。 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      speedControl:[]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_speed_control_mode2
  * @brief          速度闭环控制命令2
  * @note           主机发送该命令以控制电机的速度， 同时带有力矩限制。控制值speedControl为int32_t类型，对应
  *                 实际转速为0.01dps/LSB；控制值iqControl为int16_t类型，数值范围-2048~ 2048，对应MF电机实际转矩
  *                 电流范围-16.5A~16.5A，对应 MG 电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电机而异
  * 
  *                 1.该命令下电机的speedControl由上位机中的Max Speed值限制。
  *                 2.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xAD）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      speedControl:[]
  * @param[in]      iqControl:[-2048, 2048]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_multiposition_control_mode1
  * @brief          多圈位置闭环控制命令1
  * @note           主机发送该命令以控制电机的位置（多圈角度）。控制值angleControl为int32_t类型，对应实际位置
  *                 为0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
  * 
  *                 1.该命令下的控制值angleControl受上位机中的Max Angle值限制。 
  *                 2.该命令下电机的最大速度由上位机中的Max Speed值限制。 
  *                 3.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制 
  *                 4.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                  电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA3）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleControl:[]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_multiposition_control_mode2
  * @brief          多圈位置闭环控制命令2
  * @note           主机发送该命令以控制电机的位置（多圈角度），控制值angleControl 为int32_t 类型，对应实际位置
  *                 为0.01degree/LSB，即 36000代表360°，电机转动方向由目标位置和当前位置的差值决定 
  *                 控制值maxSpeed限制了电机转动的最大速度，为uint16_t类型，对应实际转速1dps/LSB，即360
  *                 代表360dps。
                    
  *                 1.该命令下的控制值angleControl受上位机中的Max Angle值限制。
  *                 2.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。 
  *                 3.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                  电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA4）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      maxSpeed：[]
  * @retval         none
  */
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



/**
  * @name           LKMTECH_motor_singleposition_control_mode1
  * @brief          单圈位置闭环控制命令1 
  * @note           主机发送该命令以控制电机的位置（单圈角度）
  *                 控制值spinDirection 设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针
  *                 控制值angleControl 为uint32_t 类型，对应实际位置为0.01degree/LSB，即36000代表360°。
            
  *                 1.该命令下电机的最大速度由上位机中的Max Speed值限制。
  *                 2.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。  
  *                 3.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA5）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      spinDirection:0x00 顺时针
  *                               0x01 逆时针
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_singleposition_control_mode2
  * @brief          单圈位置闭环控制命令2 
  * @note           主机发送该命令以控制电机的位置（单圈角度）
  *                 控制值spinDirection 设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针
  *                 控制值angleControl 为uint32_t 类型，对应实际位置为0.01degree/LSB，即36000代表360°。
  *                 速度控制值maxSpeed 限制了电机转动的最大速度，为uint16_t 类型，对应实际转速1dps/LSB，即360代表360dps。
  *      
  *                 1.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。 
  *                 2.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA6）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      spinDirection:0x00 顺时针
  *                               0x01 逆时针
  * @param[in]      maxSpeed:[]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_incrementposition_control_mode1
  * @brief          增量位置闭环控制命令1 
  * @note           主机发送该命令以控制电机的位置增量。 
  *                 控制值angleIncrement 为 int32_t 类型，对应实际位置为0.01degree/LSB，即 36000 代表 360°，
  *                 电机的转动方向由该参数的符号决定。
  *      
  *                 1.该命令下电机的最大速度由上位机中的Max Speed值限制。
  *                 2.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。 
  *                 3.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA7）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleIncrement:[]
  * @retval         none
  */
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


/**
  * @name           LKMTECH_motor_incrementposition_control_mode2
  * @brief          增量位置闭环控制命令2 
  * @note           主机发送该命令以控制电机的位置增量。 
  *                 控制值angleIncrement 为 int32_t 类型，对应实际位置为0.01degree/LSB，即 36000 代表 360°，
  *                 电机的转动方向由该参数的符号决定。
  *                 控制值maxSpeed限制了电机转动的最大速度，为uint32_t类型，对应实际转速1dps/LSB，即 360代表360dps。
  * 
  *                 1.该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。 
  *                 2.该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                   MS电机的最大功率由上位机中的Max Power值限制。 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA8）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      angleIncrement:[]
  * @retval         none
  */
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
