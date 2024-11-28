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


typedef struct{

  uint8_t id;//设备id号

  struct{
    int16_t speed;//电机速度
    uint16_t encoder;//电机编码器值
    int16_t iq;//转矩电流
    int8_t temperature;//电机温度
    int16_t voltage;//母线电压
    int16_t current;//母线电流
    int16_t iA;//A相电流
    int16_t iB;//B相电流
    int16_t iC;//C相电流
  }baseData;

  struct{
    uint8_t mode;//电机控制模式
    uint16_t ctlFrequency;//电机控制频率
    uint16_t revFrequency;//电机baseData采样频率
  }controlData;

  uint8_t motorState;//电机状态
  uint8_t errorState;//电机错误标志

}LKMTECH_DataHandleTypeDef;

/**
  * @name           LKMTECH_motor_init
  * @brief          瓴控电机初始化函数
  * @param[in]      device:瓴控电机句柄
  * @param[in]      id:设备CAN ID，can报文标准ID 0x140 + 设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_init(LKMTECH_DataHandleTypeDef *device, uint8_t id);

/**
  * @name           LKMTECH_motor_canrx_communication
  * @brief          瓴控电机can数据包解析函数
  * @param[in]      device:瓴控电机句柄
  * @param[in]      canrx_data:can接收报文
  * @retval         none
  */
void LKMTECH_motor_canrx_communication(LKMTECH_DataHandleTypeDef *device, uint8_t* canrx_data);


/**
  * @name           LKMTECH_motor_cantx_communication
  * @brief          电机 CAN报文 发送函数 
  * @note           电机通信为CAN 2.0通信接口，波特率1Mbps，采用标准帧格式
  * @param[in]      id:设备CAN ID，can报文标准ID 0x140 + 设备id(1~32)
  * @param[in]      data:数据指针，can报文数据位 Byte0~Byte7
  * @retval         none
  */
void LKMTECH_motor_cantx_communication(uint8_t id, uint8_t* data);



/**
  * @name           LKMTECH_read_motor_state
  * @brief          读取电机状态
  * @note           电机状态信息分为3类（state1, state2, state3）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      state_type:状态信息
  * @retval         none
  */
void LKMTECH_read_motor_state(uint8_t id, uint8_t state_type);


/**
  * @name           LKMTECH_clear_motor_error
  * @brief          清除电机错误标志命令
  * @note           电机在收到命令后回复主机。回复数据和读取电机状态1和错误标志命令相同（仅命令字节DATA[0]不同，这里为0x9B）
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_clear_motor_error(uint8_t id);


/**
  * @name           LKMTECH_motor_disable
  * @brief          电机失能
  * @note           将电机从开启状态（上电后默认状态）切换到关闭状态，
  *                 清除电机转动圈数及之前接收的控制指令，LED由常亮转为慢闪。
  *                 此时电机仍然可以回复控制命令，但不会执行动作。
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_disable(uint8_t id);



/**
  * @name           LKMTECH_motor_enable
  * @brief          电机使能
  * @note           将电机从关闭状态切换到开启状态，LED由慢闪转为常亮。
  *                 此时再发送控制指令即可控制电机动作。
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_enable(uint8_t id);



/**
  * @name           LKMTECH_motor_free
  * @brief          停止电机
  * @note           停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @retval         none
  */
void LKMTECH_motor_free(uint8_t id);



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
void LKMTECH_motor_brake(uint8_t id, uint8_t brakeControl);


/**
  * @name           LKMTECH_motor_openloop_control_mode
  * @brief          开环控制命令
  * @note           主机发送该命令以控制输出到电机的开环电压，控制值powerControl
  *                 为int16_t类型，数值范围-850~850，（电机电流和扭矩因电机而异）。 
  *                （该命令仅在MS电机上实现，其他电机无效）
  *                 该命令中的控制值powerControl不受上位机中的Max Power值限制。 
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      brakeControl:0x00：抱闸器断电，刹车启动 
  *                              0x01：抱闸器通电，刹车释放
  *                              0x10：读取抱闸器状态 
  * @retval         none
  */
void LKMTECH_motor_openloop_control_mode(uint8_t id, int16_t powerControl);



/**
  * @name           LKMTECH_motor_torque_control_mode
  * @brief          转矩闭环控制命
  * @note           主机发送该命令以控制电机的转矩电流输出，控制值iqControl为int16_t类型，数值范围-2048~ 2048，
  *                 对应MF电机实际转矩电流范围-16.5A~16.5A，对应MG电机实际转矩电流范围-33A~33A，母线电流和电机
  *                 的实际扭矩因不同电机而异。 
  *                （该命令仅在MF、MH、MG电机上实现） 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      iqControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_torque_control_mode(uint8_t id, int16_t iqControl);


/**
  * @name           LKMTECH_motor_speed_control_mode1
  * @brief          速度闭环控制命令1
  * @note           主机发送该命令以控制电机的速度， 控制值 speedControl 为 int32_t 类型，对应实际转速为0.01dps/LSB。
  *                 该命令下电机的speedControl由上位机中的Max Speed值限制
  *                 该控制模式下，电机的最大加速度由上位机中的Max Acceleration值限制。
  *                 该控制模式下，MF、MH、MG电机的最大转矩电流由上位机中的Max Torque Current值限制；
  *                 MS电机的最大功率由上位机中的Max Power值限制。
  *                 电机在收到命令后回复主机。电机回复数据和读取电机状态2命令相同（仅命令字节DATA[0]不同，这里为0xA2）。 
  * @param[in]      id:设备CAN ID,设备id(1~32)
  * @param[in]      speedControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_speed_control_mode1(uint8_t id, int32_t speedControl);



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
  * @param[in]      speedControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_speed_control_mode2(uint8_t id, int32_t speedControl, int16_t iqControl);



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
void LKMTECH_motor_multiposition_control_mode1(uint8_t id, int32_t angleControl);



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
void LKMTECH_motor_multiposition_control_mode2(uint8_t id, int32_t angleControl, uint16_t maxSpeed);




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
void LKMTECH_motor_singleposition_control_mode1(uint8_t id, uint32_t angleControl, uint8_t spinDirection);



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
void LKMTECH_motor_singleposition_control_mode2(uint8_t id, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed);


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
void LKMTECH_motor_incrementposition_control_mode1(uint8_t id, int32_t angleIncrement);



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
