#include "test_MF9025V2.h"


LKMTECH_DataHandleTypeDef MF9025V2_motor;


int16_t t_iq = 0;
int32_t t_speed = 0;
int32_t t_angle = 0;
uint32_t t_uangle = 0;
uint8_t t_spin = 0;
uint32_t t_uspeed = 0;
int32_t t_iangle = 0;
/*success*/
void MF9025V2_init(void)
{
	//t_iqControl = 0;
	LKMTECH_motor_init(&MF9025V2_motor, 0x01);
}

/*success*/
void MF9025V2_torque_test(void)
{
	LKMTECH_motor_torque_control_mode(MF9025V2_motor.id, t_iq);
}

/*success*/
void MF9025V2_speed_test1(void)
{
	LKMTECH_motor_speed_control_mode1(MF9025V2_motor.id, t_speed);
}

/*failure*/
void MF9025V2_speed_test2(void)
{
	LKMTECH_motor_speed_control_mode2(MF9025V2_motor.id, t_speed, t_iq);
}

/*success*/
void MF9025V2_multiposition_test1(void)
{
	LKMTECH_motor_multiposition_control_mode1(MF9025V2_motor.id, t_angle);
}

/*success*/
//当t_speed = 0时，max_speed不起作用，类似于MF9025V2_multiposition_test1现象
void MF9025V2_multiposition_test2(void)
{
	LKMTECH_motor_multiposition_control_mode2(MF9025V2_motor.id, t_angle, t_speed);
}

/*half success*/
//存在跑飞停不下来的情况
//t_spin相当于用来选择最短路径还是最长路径s
void MF9025V2_motor_singleposition_test1(void)
{
	LKMTECH_motor_singleposition_control_mode1(MF9025V2_motor.id, t_uangle, t_spin);
}

/*success*/
void MF9025V2_motor_singleposition_test2(void)
{
	LKMTECH_motor_singleposition_control_mode2(MF9025V2_motor.id, t_uangle, t_spin, t_uspeed);
}


/*success*/
void LKMTECH_motor_incrementposition_test1(void)
{
	LKMTECH_motor_incrementposition_control_mode1(MF9025V2_motor.id, t_iangle);
}

void LKMTECH_motor_incrementposition_test2(void)
{
	LKMTECH_motor_incrementposition_control_mode2(MF9025V2_motor.id, t_iangle, t_speed);
}

/*success*/
//1ms定时器，连续调用LKMTECH_read_motor_state函数一次获取电机3个状态数据，第一次和第三次可以被成果处理，第二次失败
//两条指令之间要设计一段最小发送时间间隔？
void MF9025V2_get_message_test(void)
{
	LKMTECH_read_motor_state(MF9025V2_motor.id, LKMTECH_MOTOR_STATE_ONE);
	LKMTECH_read_motor_state(MF9025V2_motor.id, LKMTECH_MOTOR_STATE_TWO);
	LKMTECH_read_motor_state(MF9025V2_motor.id, LKMTECH_MOTOR_STATE_THREE);
}

/*success*/
//母线电流值一直为零，上位机也是同一结果
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	
	LKMTECH_motor_canrx_communication(&MF9025V2_motor, rx_data);

}
