#include "auto_takeoff.h"



  u8 takeoff_state = 0; //0 disable   1 enable 
  u8 takeoff_enable = READY_FLY; 
	u8 takeoff_acc_speed_first = 180; //һ��ʼ��� 1680 - 1000 = 680 ������������ 680-500 = 180
	u8 takeoff_acc_speed_second = 180; //

	uint8_t line_tracker = 0;

