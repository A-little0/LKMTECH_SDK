/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       bsp_LKMTECH.c/h
  * @brief     
  *             ������겿ص��can�����ļ�������canͨѶ�շ�������ģʽ��λ��ģʽ
  *             �˿�ģʽ���ٶ�ģʽ����API���Լ����������ȡ��д��API��
  *             
  * @note      1.ͬһ�����Ϲ����Թ��ض��32�������߸��������������������Ϊ�˷�ֹ���߳�ͻ��
  *             ÿ��������Ҫ���ò�ͬ��ID
  *            2.���������߷��͵���������ӦID�ĵ�����յ������ִ�У�����һ��ʱ���
  *            ��0.25ms�ڣ������ط��ͻظ���
  * 
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nove-28-2024   chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
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

  uint8_t id;//�豸id��

  struct{
    int16_t speed;//����ٶ�
    uint16_t encoder;//���������ֵ
    int16_t iq;//ת�ص���
    int8_t temperature;//����¶�
    int16_t voltage;//ĸ�ߵ�ѹ
    int16_t current;//ĸ�ߵ���
    int16_t iA;//A�����
    int16_t iB;//B�����
    int16_t iC;//C�����
  }baseData;

  struct{
    uint8_t mode;//�������ģʽ
    uint16_t ctlFrequency;//�������Ƶ��
    uint16_t revFrequency;//���baseData����Ƶ��
  }controlData;

  uint8_t motorState;//���״̬
  uint8_t errorState;//��������־

}LKMTECH_DataHandleTypeDef;

/**
  * @name           LKMTECH_motor_init
  * @brief          겿ص����ʼ������
  * @param[in]      device:겿ص�����
  * @param[in]      id:�豸CAN ID��can���ı�׼ID 0x140 + �豸id(1~32)
  * @retval         none
  */
void LKMTECH_motor_init(LKMTECH_DataHandleTypeDef *device, uint8_t id);

/**
  * @name           LKMTECH_motor_canrx_communication
  * @brief          겿ص��can���ݰ���������
  * @param[in]      device:겿ص�����
  * @param[in]      canrx_data:can���ձ���
  * @retval         none
  */
void LKMTECH_motor_canrx_communication(LKMTECH_DataHandleTypeDef *device, uint8_t* canrx_data);


/**
  * @name           LKMTECH_motor_cantx_communication
  * @brief          ��� CAN���� ���ͺ��� 
  * @note           ���ͨ��ΪCAN 2.0ͨ�Žӿڣ�������1Mbps�����ñ�׼֡��ʽ
  * @param[in]      id:�豸CAN ID��can���ı�׼ID 0x140 + �豸id(1~32)
  * @param[in]      data:����ָ�룬can��������λ Byte0~Byte7
  * @retval         none
  */
void LKMTECH_motor_cantx_communication(uint8_t id, uint8_t* data);



/**
  * @name           LKMTECH_read_motor_state
  * @brief          ��ȡ���״̬
  * @note           ���״̬��Ϣ��Ϊ3�ࣨstate1, state2, state3��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      state_type:״̬��Ϣ
  * @retval         none
  */
void LKMTECH_read_motor_state(uint8_t id, uint8_t state_type);


/**
  * @name           LKMTECH_clear_motor_error
  * @brief          �����������־����
  * @note           ������յ������ظ��������ظ����ݺͶ�ȡ���״̬1�ʹ����־������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0x9B��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @retval         none
  */
void LKMTECH_clear_motor_error(uint8_t id);


/**
  * @name           LKMTECH_motor_disable
  * @brief          ���ʧ��
  * @note           ������ӿ���״̬���ϵ��Ĭ��״̬���л����ر�״̬��
  *                 ������ת��Ȧ����֮ǰ���յĿ���ָ�LED�ɳ���תΪ������
  *                 ��ʱ�����Ȼ���Իظ��������������ִ�ж�����
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @retval         none
  */
void LKMTECH_motor_disable(uint8_t id);



/**
  * @name           LKMTECH_motor_enable
  * @brief          ���ʹ��
  * @note           ������ӹر�״̬�л�������״̬��LED������תΪ������
  *                 ��ʱ�ٷ��Ϳ���ָ��ɿ��Ƶ��������
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @retval         none
  */
void LKMTECH_motor_enable(uint8_t id);



/**
  * @name           LKMTECH_motor_free
  * @brief          ֹͣ���
  * @note           ֹͣ�������������������״̬���ٴη��Ϳ���ָ��ɿ��Ƶ�������� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @retval         none
  */
void LKMTECH_motor_free(uint8_t id);



/**
  * @name           LKMTECH_motor_brake
  * @brief          ���Ʊ�բ����
  * @note           ���Ʊ�բ���Ŀ��ϣ����߶�ȡ��ǰ��բ����״̬�� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      brakeControl:0x00����բ���ϵ磬ɲ������ 
  *                              0x01����բ��ͨ�磬ɲ���ͷ�
  *                              0x10����ȡ��բ��״̬ 
  * @retval         none
  */
void LKMTECH_motor_brake(uint8_t id, uint8_t brakeControl);


/**
  * @name           LKMTECH_motor_openloop_control_mode
  * @brief          ������������
  * @note           �������͸������Կ������������Ŀ�����ѹ������ֵpowerControl
  *                 Ϊint16_t���ͣ���ֵ��Χ-850~850�������������Ť���������죩�� 
  *                �����������MS�����ʵ�֣����������Ч��
  *                 �������еĿ���ֵpowerControl������λ���е�Max Powerֵ���ơ� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      brakeControl:0x00����բ���ϵ磬ɲ������ 
  *                              0x01����բ��ͨ�磬ɲ���ͷ�
  *                              0x10����ȡ��բ��״̬ 
  * @retval         none
  */
void LKMTECH_motor_openloop_control_mode(uint8_t id, int16_t powerControl);



/**
  * @name           LKMTECH_motor_torque_control_mode
  * @brief          ת�رջ�������
  * @note           �������͸������Կ��Ƶ����ת�ص������������ֵiqControlΪint16_t���ͣ���ֵ��Χ-2048~ 2048��
  *                 ��ӦMF���ʵ��ת�ص�����Χ-16.5A~16.5A����ӦMG���ʵ��ת�ص�����Χ-33A~33A��ĸ�ߵ����͵��
  *                 ��ʵ��Ť����ͬ������졣 
  *                �����������MF��MH��MG�����ʵ�֣� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      iqControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_torque_control_mode(uint8_t id, int16_t iqControl);


/**
  * @name           LKMTECH_motor_speed_control_mode1
  * @brief          �ٶȱջ���������1
  * @note           �������͸������Կ��Ƶ�����ٶȣ� ����ֵ speedControl Ϊ int32_t ���ͣ���Ӧʵ��ת��Ϊ0.01dps/LSB��
  *                 �������µ����speedControl����λ���е�Max Speedֵ����
  *                 �ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ�
  *                 �ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                 MS��������������λ���е�Max Powerֵ���ơ�
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA2���� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      speedControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_speed_control_mode1(uint8_t id, int32_t speedControl);



/**
  * @name           LKMTECH_motor_speed_control_mode2
  * @brief          �ٶȱջ���������2
  * @note           �������͸������Կ��Ƶ�����ٶȣ� ͬʱ�����������ơ�����ֵspeedControlΪint32_t���ͣ���Ӧ
  *                 ʵ��ת��Ϊ0.01dps/LSB������ֵiqControlΪint16_t���ͣ���ֵ��Χ-2048~ 2048����ӦMF���ʵ��ת��
  *                 ������Χ-16.5A~16.5A����Ӧ MG ���ʵ��ת�ص�����Χ-33A~33A��ĸ�ߵ����͵����ʵ��Ť����ͬ�������
  * 
  *                 1.�������µ����speedControl����λ���е�Max Speedֵ���ơ�
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ�
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xAD��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      speedControl:[-2048, 2048]
  * @retval         none
  */
void LKMTECH_motor_speed_control_mode2(uint8_t id, int32_t speedControl, int16_t iqControl);



/**
  * @name           LKMTECH_motor_multiposition_control_mode1
  * @brief          ��Ȧλ�ñջ���������1
  * @note           �������͸������Կ��Ƶ����λ�ã���Ȧ�Ƕȣ�������ֵangleControlΪint32_t���ͣ���Ӧʵ��λ��
  *                 Ϊ0.01degree/LSB���� 36000 ���� 360�㣬���ת��������Ŀ��λ�ú͵�ǰλ�õĲ�ֵ������
  * 
  *                 1.�������µĿ���ֵangleControl����λ���е�Max Angleֵ���ơ� 
  *                 2.�������µ��������ٶ�����λ���е�Max Speedֵ���ơ� 
  *                 3.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���� 
  *                 4.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ����
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                  ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA3��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      angleControl:[]
  * @retval         none
  */
void LKMTECH_motor_multiposition_control_mode1(uint8_t id, int32_t angleControl);



/**
  * @name           LKMTECH_motor_multiposition_control_mode2
  * @brief          ��Ȧλ�ñջ���������2
  * @note           �������͸������Կ��Ƶ����λ�ã���Ȧ�Ƕȣ�������ֵangleControl Ϊint32_t ���ͣ���Ӧʵ��λ��
  *                 Ϊ0.01degree/LSB���� 36000����360�㣬���ת��������Ŀ��λ�ú͵�ǰλ�õĲ�ֵ���� 
  *                 ����ֵmaxSpeed�����˵��ת��������ٶȣ�Ϊuint16_t���ͣ���Ӧʵ��ת��1dps/LSB����360
  *                 ����360dps��
                    
  *                 1.�������µĿ���ֵangleControl����λ���е�Max Angleֵ���ơ�
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ� 
  *                 3.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                  ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA4��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      maxSpeed��[]
  * @retval         none
  */
void LKMTECH_motor_multiposition_control_mode2(uint8_t id, int32_t angleControl, uint16_t maxSpeed);




/**
  * @name           LKMTECH_motor_singleposition_control_mode1
  * @brief          ��Ȧλ�ñջ���������1 
  * @note           �������͸������Կ��Ƶ����λ�ã���Ȧ�Ƕȣ�
  *                 ����ֵspinDirection ���õ��ת���ķ���Ϊuint8_t���ͣ�0x00����˳ʱ�룬0x01������ʱ��
  *                 ����ֵangleControl Ϊuint32_t ���ͣ���Ӧʵ��λ��Ϊ0.01degree/LSB����36000����360�㡣
            
  *                 1.�������µ��������ٶ�����λ���е�Max Speedֵ���ơ�
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ�  
  *                 3.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA5��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      spinDirection:0x00 ˳ʱ��
  *                               0x01 ��ʱ��
  * @retval         none
  */
void LKMTECH_motor_singleposition_control_mode1(uint8_t id, uint32_t angleControl, uint8_t spinDirection);



/**
  * @name           LKMTECH_motor_singleposition_control_mode2
  * @brief          ��Ȧλ�ñջ���������2 
  * @note           �������͸������Կ��Ƶ����λ�ã���Ȧ�Ƕȣ�
  *                 ����ֵspinDirection ���õ��ת���ķ���Ϊuint8_t���ͣ�0x00����˳ʱ�룬0x01������ʱ��
  *                 ����ֵangleControl Ϊuint32_t ���ͣ���Ӧʵ��λ��Ϊ0.01degree/LSB����36000����360�㡣
  *                 �ٶȿ���ֵmaxSpeed �����˵��ת��������ٶȣ�Ϊuint16_t ���ͣ���Ӧʵ��ת��1dps/LSB����360����360dps��
  *      
  *                 1.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ� 
  *                 2.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA6��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      angleControl:[]
  * @param[in]      spinDirection:0x00 ˳ʱ��
  *                               0x01 ��ʱ��
  * @param[in]      maxSpeed:[]
  * @retval         none
  */
void LKMTECH_motor_singleposition_control_mode2(uint8_t id, uint32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed);


/**
  * @name           LKMTECH_motor_incrementposition_control_mode1
  * @brief          ����λ�ñջ���������1 
  * @note           �������͸������Կ��Ƶ����λ�������� 
  *                 ����ֵangleIncrement Ϊ int32_t ���ͣ���Ӧʵ��λ��Ϊ0.01degree/LSB���� 36000 ���� 360�㣬
  *                 �����ת�������ɸò����ķ��ž�����
  *      
  *                 1.�������µ��������ٶ�����λ���е�Max Speedֵ���ơ�
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ� 
  *                 3.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA7��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      angleIncrement:[]
  * @retval         none
  */
void LKMTECH_motor_incrementposition_control_mode1(uint8_t id, int32_t angleIncrement);



/**
  * @name           LKMTECH_motor_incrementposition_control_mode2
  * @brief          ����λ�ñջ���������2 
  * @note           �������͸������Կ��Ƶ����λ�������� 
  *                 ����ֵangleIncrement Ϊ int32_t ���ͣ���Ӧʵ��λ��Ϊ0.01degree/LSB���� 36000 ���� 360�㣬
  *                 �����ת�������ɸò����ķ��ž�����
  *                 ����ֵmaxSpeed�����˵��ת��������ٶȣ�Ϊuint32_t���ͣ���Ӧʵ��ת��1dps/LSB���� 360����360dps��
  * 
  *                 1.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ� 
  *                 2.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA8��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
