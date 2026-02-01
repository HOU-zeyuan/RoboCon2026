#include "board_communicate.h"
// 是否完成任务标志位
uint8_t flag_up_step;	// 上台阶
uint8_t flag_down_step; // 下台阶
uint8_t flag_up_R1;		// 上R1
// 与另一辆车通讯
uint8_t flag_D_init;		   // D初始化
uint8_t flag_D_ready_get_head; // D准备取武器头
uint8_t flag_D_get_left;	   // D左臂取武器头
uint8_t flag_D_get_right;	   // D右臂取武器头
uint8_t flag_D_get_block;	   // D准备存S给的方块

uint8_t flag_S_init;			  // S初始化
uint8_t flag_S_get_block_down;	  // S取下层方块
uint8_t flag_S_get_block_up;	  // S取上层方块
uint8_t flag_S_get_block_top;	  // S取上上层方块
uint8_t flag_S_throw_block_back;  // S向后扔方块
uint8_t flag_S_throw_block_front; // S向前扔方块
uint8_t flag_S_give_D;			  // S给D方块
uint8_t flag_S_get_D_block;		  // S取D给的方块
uint8_t flag_S_put_block_middle;  // S三区放中层方块
uint8_t flag_S_put_block_top;	  // S三区放上层方块

uint8_t flag_S_hold_block;
bool flag_update_tx = true;
/*******************************************************************************************
函数名称：Upper_Lower_Communication
函数功能：进行上下板通讯 下板将数据发送至上板
输入：
		对txData进行赋值
输出：

备注：D指双臂，S指单臂
		[0] 帧头 0x88
		[1] D初始化
		[2] D准备取武器头
		[3] D左臂取武器
		[4] D右臂取武器
		[5] D准备存S给的方块

		[6] S初始化
		[7] S取下层方块
		[8] S取上层方块
		[9] S取上上层方块
		[10]S向后扔方块
		[11]S向前扔方块
		[12]S给D方块
		[13]S取D给的方块
		[14]S三区放中层方块
		[15]S三区放上层方块
		[16]帧尾 0x66

*******************************************************************************************/
void Upper_Lower_Communication(void)
{
	down_tx[0] = 0x88; // 帧头
	down_tx[16] = 0x66;
	updateTxData();								 // 1-15位
	HAL_UART_Transmit_DMA(&huart1, down_tx, 17); // 发送
}
/**************************上板数据解算**************************************/
int CHM;
void Upper_Data_Recieve_Deal(void)
{
	if (down_rx[0] != 0x33 && down_rx[16] != 0x55) // 帧头帧尾检验
	{
		CHM++;
		return;
	}
	//		system_monitor_up[0] = rxData[2];
	//		system_monitor_up[1] = rxData[3];
	//		//2 3位上板为故障error标志位
	// 上板是否完成任务

	flag_D_init = down_rx[1];			// D初始化
	flag_D_ready_get_head = down_rx[2]; // D准备取武器头
	flag_D_get_left = down_rx[3];		// D取左武器头
	flag_D_get_right = down_rx[4];		// D取右武器头
	flag_D_get_block = down_rx[5];		// D准备存S给的方块
	// flag_D_give_s_block= down_rx[6];//D给S方块

	flag_S_init = down_rx[6];				// S初始化
	flag_S_get_block_down = down_rx[7];		// S取下层方块
	flag_S_get_block_up = down_rx[8];		// S取上层方块
	flag_S_get_block_top = down_rx[9];		// S取上上层方块
	flag_S_throw_block_back = down_rx[10];	// S向后扔方块
	flag_S_throw_block_front = down_rx[11]; // S向前扔方块
	flag_S_give_D = down_rx[12];			// S给D方块
	flag_S_get_D_block = down_rx[13];		// S取D给的方块
	flag_S_put_block_middle = down_rx[14];	// S三区放中层方块
	flag_S_put_block_top = down_rx[15];		// S三区放上层方块

	// flag_S_put_block = down_rx[10];
	// my_system_monitor.rate_cnt.UPPER_TO_LOWER++;
}
/****************************更新发送数组***************************/
void updateTxData(void)
{
	down_tx[1] = 0;
	down_tx[2] = 0;
	down_tx[3] = 0;
	down_tx[4] = 0;
	down_tx[5] = 0;

	down_tx[6] = 0;
	down_tx[7] = 0;
	down_tx[8] = 0;
	down_tx[9] = 0;
	down_tx[10] = 0;
	down_tx[11] = 0;
	down_tx[12] = 0;
	down_tx[13] = 0;
	down_tx[14] = 0;
	down_tx[15] = 0;
	// down_tx[16] = 0;

	// 根据当前状态修改对应的down_tx位为1
	switch (up_d_state)
	{
	case D_INIT: // 1
		down_tx[1] = 1;
		break;
	case D_READY_GET_HEAD: // 2
		down_tx[2] = 1;
		break;
	case D_GET_LEFT: // 3
		down_tx[3] = 1;
		break;
	case D_GET_RIGHT: // 4
		down_tx[4] = 1;
		break;
	case D_GET_BLOCK: // 5
		down_tx[5] = 1;
		break;

	default:
		down_tx[1] = 0;
		down_tx[2] = 0;
		down_tx[3] = 0;
		down_tx[4] = 0;
		down_tx[5] = 0;

		break;
	}
	switch (up_s_state)
	{
	case S_INIT: // 0
		down_tx[6] = 1;
		break;
	case S_GET_BLOCK_DOWN: // 1
		down_tx[7] = 1;
		break;
	case S_GET_BLOCK_UP: // 2
		down_tx[8] = 1;
		break;
	case S_GET_BLOCK_TOP: // 3
		down_tx[9] = 1;
		break;
	case S_THROW_BLOCK_BACK: // 4
		down_tx[10] = 1;
		break;
	case S_THROW_BLOCK_FRONT: // 5
		down_tx[11] = 1;
		break;
	case S_GIVE_D: // 6
		down_tx[12] = 1;
		break;
	case S_GET_D_BLOCK: // 7
		down_tx[13] = 1;
		break;
	case S_PUT_BLOCK_MIDDLE: // 8
		down_tx[14] = 1;
		break;
	case S_PUT_BLOCK_TOP: // 9
		down_tx[15] = 1;
		break;

	default:
		down_tx[7] = 0;
		down_tx[8] = 0;
		down_tx[9] = 0;
		down_tx[10] = 0;
		down_tx[11] = 0;
		down_tx[12] = 0;
		down_tx[13] = 0;
		down_tx[14] = 0;
		down_tx[15] = 0;

		break;
	}
}
/************************CRC-16-IBM多项式：x^16 + x^15 + x^2 + 1 (0xA001)**********************/
uint16_t crc16(uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF; // 初始值

	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
			{
				crc = (crc >> 1) ^ 0xA001; // 多项式反转值
			}
			else
			{
				crc = crc >> 1;
			}
		}
	}
	return crc;
}
