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
#include "bsp_LKMTECH.h"
#include "can.h"

CAN_TxHeaderTypeDef lkmtech_tx_message;
static uint8_t lkmtech_can_send_data[8];
static uint8_t lkmtech_can_receive_data[8];


/**
  * @name           LKMTECH_motor_init
  * @brief          겿ص����ʼ������
  * @param[in]      device:겿ص�����
  * @param[in]      id:�豸CAN ID��can���ı�׼ID 0x140 + �豸id(1~32)
  * @retval         none
  */
void LKMTECH_motor_init(LKMTECH_DataHandleTypeDef *device, uint8_t id)
{
  /*��ʼ��id*/
  device->id = id;
  /*��ʼ��base data*/
  device->baseData.speed = 0;
  device->baseData.encoder = 0;
  device->baseData.iq = 0;
  device->baseData.current = 0;
  device->baseData.voltage = 0;
  device->baseData.temperature = 0;
  /*��ʼ��control data*/
  device->controlData.mode = 0;
  device->controlData.ctlFrequency = 0;
  device->controlData.revFrequency = 0;
  /*��ʼ��*/
  device->motorState = 0;
  device->errorState = 0;
}


/**
  * @name           LKMTECH_motor_canrx_communication
  * @brief          겿ص��can���ݰ���������
  * @param[in]      device:겿ص�����
  * @param[in]      canrx_data:can���ձ���
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
  * @brief          ��� CAN���� ���ͺ��� 
  * @note           ���ͨ��ΪCAN 2.0ͨ�Žӿڣ�������1Mbps�����ñ�׼֡��ʽ
  * @param[in]      id:�豸CAN ID��can���ı�׼ID 0x140 + �豸id(1~32)
  * @param[in]      data:����ָ�룬can��������λ Byte0~Byte7
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
  * @brief          ��ȡ���״̬
  * @note           ���״̬��Ϣ��Ϊ3�ࣨstate1, state2, state3��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      state_type:״̬��Ϣ
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
    /*�������ȡ��ǰ������¶ȡ���ѹ�ʹ���״̬��־*/
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE1_CMD;
  }
  else if(state_type == 1)
  {
    /*�������ȡ��ǰ������¶ȡ����ת�ص�����MF��MG��/���������ʣ�MS����ת�١�������λ*/
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE2_CMD;
  }
  else
  {
    /*����MS���û���������������������MS����������á�
    �������ȡ��ǰ������¶Ⱥ�3��������� */ 
    lkmtech_can_send_data[0] = LKMTECH_READ_MOTOR_STATE3_CMD;
  }

  LKMTECH_motor_cantx_communication(id, lkmtech_can_send_data);
}



/**
  * @name           LKMTECH_clear_motor_error
  * @brief          �����������־����
  * @note           ������յ������ظ��������ظ����ݺͶ�ȡ���״̬1�ʹ����־������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0x9B��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          ���ʧ��
  * @note           ������ӿ���״̬���ϵ��Ĭ��״̬���л����ر�״̬��
  *                 ������ת��Ȧ����֮ǰ���յĿ���ָ�LED�ɳ���תΪ������
  *                 ��ʱ�����Ȼ���Իظ��������������ִ�ж�����
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          ���ʹ��
  * @note           ������ӹر�״̬�л�������״̬��LED������תΪ������
  *                 ��ʱ�ٷ��Ϳ���ָ��ɿ��Ƶ��������
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          ֹͣ���
  * @note           ֹͣ�������������������״̬���ٴη��Ϳ���ָ��ɿ��Ƶ�������� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          ���Ʊ�բ����
  * @note           ���Ʊ�բ���Ŀ��ϣ����߶�ȡ��ǰ��բ����״̬�� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
  * @param[in]      brakeControl:0x00����բ���ϵ磬ɲ������ 
  *                              0x01����բ��ͨ�磬ɲ���ͷ�
  *                              0x10����ȡ��բ��״̬ 
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
  * @brief          ������������
  * @note           �������͸������Կ������������Ŀ�����ѹ������ֵpowerControl
  *                 Ϊint16_t���ͣ���ֵ��Χ-850~850�������������Ť���������죩
  *                �����������MS�����ʵ�֣����������Ч��
  * 
  *                 1.�������еĿ���ֵpowerControl������λ���е�Max Powerֵ���� 
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          ת�رջ�������
  * @note           �������͸������Կ��Ƶ����ת�ص������������ֵiqControlΪint16_t���ͣ���ֵ��Χ-2048~ 2048��
  *                 ��ӦMF���ʵ��ת�ص�����Χ-16.5A~16.5A����ӦMG���ʵ��ת�ص�����Χ-33A~33A��ĸ�ߵ����͵��
  *                 ��ʵ��Ť����ͬ������졣 
  *                �����������MF��MH��MG�����ʵ�֣�
  *               
  *                 1. �������еĿ���ֵiqControl������λ���е�Max Torque Currentֵ����
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ����Ϊ0xA1��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          �ٶȱջ���������1
  * @note           �������͸������Կ��Ƶ�����ٶȣ� ����ֵ speedControl Ϊ int32_t ���ͣ���Ӧʵ��ת��Ϊ0.01dps/LSB��
  * 
  *                 1.�������µ����speedControl����λ���е�Max Speedֵ����
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ�
  *                 3.�ÿ���ģʽ�£�MF��MH��MG��������ת�ص�������λ���е�Max Torque Currentֵ���ƣ�
  *                   MS��������������λ���е�Max Powerֵ���ơ�
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xA2���� 
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
  * @brief          �ٶȱջ���������2
  * @note           �������͸������Կ��Ƶ�����ٶȣ� ͬʱ�����������ơ�����ֵspeedControlΪint32_t���ͣ���Ӧ
  *                 ʵ��ת��Ϊ0.01dps/LSB������ֵiqControlΪint16_t���ͣ���ֵ��Χ-2048~ 2048����ӦMF���ʵ��ת��
  *                 ������Χ-16.5A~16.5A����Ӧ MG ���ʵ��ת�ص�����Χ-33A~33A��ĸ�ߵ����͵����ʵ��Ť����ͬ�������
  * 
  *                 1.�������µ����speedControl����λ���е�Max Speedֵ���ơ�
  *                 2.�ÿ���ģʽ�£�����������ٶ�����λ���е�Max Accelerationֵ���ơ�
  *                 ������յ������ظ�����������ظ����ݺͶ�ȡ���״̬2������ͬ���������ֽ�DATA[0]��ͬ������Ϊ0xAD��
  * @param[in]      id:�豸CAN ID,�豸id(1~32)
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
