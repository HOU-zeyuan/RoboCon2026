#include "vofa.h"

VOFA_DATA vofa_data;

const uint8_t FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F}; // 固定终止符

// 使用justfloat协议将数据发送至vofa+上位机
void VOFA_transmit_data(float *data, uint8_t num)
{
    memcpy(vofa_data.ch_data, data, num * sizeof(float));

    // 明确填充剩余通道为0
    for (int i = num; i < CH_COUNT; i++){
        vofa_data.ch_data[i] = 0.0f;
    }

    memcpy(vofa_data.tail, FRAME_TAIL, sizeof(FRAME_TAIL));

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&vofa_data, sizeof(VOFA_DATA)); // 发送
}

void VOFA_pack_data(){
    vofa[0] = nav.expect_robot_global_velt.fpY;
    vofa[1] = dt35_now.dt35_voltage_1;
    vofa[2] = chassis_run.leftup.fpDes;
    vofa[3] = leftup_motor.anglev;

    vofa[4] = chassis_run.rightup.fpDes;
    vofa[5] = chassis_run.rightup.fpU;
    vofa[6] = chassis_run.leftdown.fpDes;
    vofa[7] = chassis_run.leftdown.fpU;
    vofa[8] = chassis_run.rightdown.fpDes;
    vofa[9] = chassis_run.rightdown.fpU;
    vofa[10] = chassis_run.leftup.fpDes;
    vofa[11] = chassis_run.leftup.fpU;
    vofa[12] = chassis_run.rightup.fpFB;
    vofa[13] = chassis_run.leftdown.fpFB;
    vofa[14] = chassis_run.rightdown.fpFB;
    vofa[15] = chassis_run.leftup.fpFB;
}
