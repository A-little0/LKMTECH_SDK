#ifndef __TEST_MF9025V2_H
#define __TEST_MF9025V2_H

#include "bsp_LKMTECH.h"
#include "can.h"

extern LKMTECH_DataHandleTypeDef MF9025V2_motor;


void MF9025V2_init(void);
void MF9025V2_torque_test(void);
void MF9025V2_speed_test1(void);
void MF9025V2_speed_test2(void);
void MF9025V2_multiposition_test1(void);
void MF9025V2_multiposition_test2(void);
void MF9025V2_motor_singleposition_test1(void);
void MF9025V2_motor_singleposition_test2(void);
void LKMTECH_motor_incrementposition_test1(void);
void LKMTECH_motor_incrementposition_test2(void);
void MF9025V2_get_message_test(void);
#endif
