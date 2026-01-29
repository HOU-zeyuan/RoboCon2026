#include "locate.h"

void Vision_location(ST_Nav *p_nav)
{
    p_nav->auto_path.pos_pid.x.fpFB=vision_data_recieve.x1;
	  p_nav->auto_path.pos_pid.y.fpFB=vision_data_recieve.y1;
	  p_nav->auto_path.pos_pid.z.fpFB=vision_data_recieve.z1;
	  stRobot.stPos.fpPosQ=vision_data_recieve.yaw1*1800/3.1415926;
	  p_nav->auto_path.pos_pid.w.fpFB=vision_data_recieve.yaw1*180/3.1415926;
	  switch(vision_data_recieve.id)
    {
			case 1:
				vision_data_recieve.real_z=MEIHUA_1;
				break;
			case 2:
				vision_data_recieve.real_z=MEIHUA_2;
				break;
			case 3:
				vision_data_recieve.real_z=MEIHUA_3;
				break;
			case 4:
				vision_data_recieve.real_z=MEIHUA_4;
				break;
			case 5:
				vision_data_recieve.real_z=MEIHUA_5;
				break;
			case 6:
				vision_data_recieve.real_z=MEIHUA_6;
				break;
			case 7:
				vision_data_recieve.real_z=MEIHUA_7;
				break;
			case 8:
				vision_data_recieve.real_z=MEIHUA_8;
				break;
			case 9:
				vision_data_recieve.real_z=MEIHUA_9;
				break;
			case 10:
				vision_data_recieve.real_z=MEIHUA_10;
				break;
			case 11:
				vision_data_recieve.real_z=MEIHUA_11;
				break;
			case 12:
				vision_data_recieve.real_z=MEIHUA_12;
				break;
		}
		switch(vision_data_recieve.next_id)
    {
			case 1:
				vision_data_recieve.z2=MEIHUA_1;
				break;
			case 2:
				vision_data_recieve.z2=MEIHUA_2;
				break;
			case 3:
				vision_data_recieve.z2=MEIHUA_3;
				break;
			case 4:
				vision_data_recieve.z2=MEIHUA_4;
				break;
			case 5:
				vision_data_recieve.z2=MEIHUA_5;
				break;
			case 6:
				vision_data_recieve.z2=MEIHUA_6;
				break;
			case 7:
				vision_data_recieve.z2=MEIHUA_7;
				break;
			case 8:
				vision_data_recieve.z2=MEIHUA_8;
				break;
			case 9:
				vision_data_recieve.z2=MEIHUA_9;
				break;
			case 10:
				vision_data_recieve.z2=MEIHUA_10;
				break;
			case 11:
				vision_data_recieve.z2=MEIHUA_11;
				break;
			case 12:
				vision_data_recieve.z2=MEIHUA_12;
				break;
		}
		switch(vision_data_recieve.extra_id1)
    {
			case 1:
				vision_data_recieve.z3=MEIHUA_1;
				break;
			case 2:
				vision_data_recieve.z3=MEIHUA_2;
				break;
			case 3:
				vision_data_recieve.z3=MEIHUA_3;
				break;
			case 4:
				vision_data_recieve.z3=MEIHUA_4;
				break;
			case 5:
				vision_data_recieve.z3=MEIHUA_5;
				break;
			case 6:
				vision_data_recieve.z3=MEIHUA_6;
				break;
			case 7:
				vision_data_recieve.z3=MEIHUA_7;
				break;
			case 8:
				vision_data_recieve.z3=MEIHUA_8;
				break;
			case 9:
				vision_data_recieve.z3=MEIHUA_9;
				break;
			case 10:
				vision_data_recieve.z3=MEIHUA_10;
				break;
			case 11:
				vision_data_recieve.z3=MEIHUA_11;
				break;
			case 12:
				vision_data_recieve.z3=MEIHUA_12;
				break;
		}
		switch(vision_data_recieve.extra_id2)
    {
			case 1:
				vision_data_recieve.z4=MEIHUA_1;
				break;
			case 2:
				vision_data_recieve.z4=MEIHUA_2;
				break;
			case 3:
				vision_data_recieve.z4=MEIHUA_3;
				break;
			case 4:
				vision_data_recieve.z4=MEIHUA_4;
				break;
			case 5:
				vision_data_recieve.z4=MEIHUA_5;
				break;
			case 6:
				vision_data_recieve.z4=MEIHUA_6;
				break;
			case 7:
				vision_data_recieve.z4=MEIHUA_7;
				break;
			case 8:
				vision_data_recieve.z4=MEIHUA_8;
				break;
			case 9:
				vision_data_recieve.z4=MEIHUA_9;
				break;
			case 10:
				vision_data_recieve.z4=MEIHUA_10;
				break;
			case 11:
				vision_data_recieve.z4=MEIHUA_11;
				break;
			case 12:
				vision_data_recieve.z4=MEIHUA_12;
				break;
		}
}
