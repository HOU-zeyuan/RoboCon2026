#include "chassis.h"

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav)
{
    float fpQ;
    if (p_nav->nav_state == RC_LOCAL ||p_nav->nav_state==SEMIAUTO_UP_DOWN_STAIRS)
    {   // 以车身坐标系操控
        fpQ = PI/2;
    }
	else{
        // 以全场坐标系操控
        fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
    }
    Convert_velt(&p_nav->expect_robot_global_velt, &expect_robot_local_Velt, fpQ); // 把local->fpW = global->fpW注释了

    // 角速度死区
    if (fabs(expect_robot_local_Velt.fpW) < 0.1f) // rad/s
        expect_robot_local_Velt.fpW = 0.f;

    // 左右侧电机转向正方向相反！！
    steer_velt.leftup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightdown = powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.leftdown = -powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;

    steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.leftdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY - DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.rightdown = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(DOWN_ANGLE), expect_robot_local_Velt.fpY + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(DOWN_ANGLE)) / PI * 180.f;
    // steer_pos.leftup = normalize_angle(steer_pos.leftup);
    // steer_pos.leftdown = normalize_angle(steer_pos.leftdown);
    // steer_pos.rightup = normalize_angle(steer_pos.rightup);
    // steer_pos.rightdown = normalize_angle(steer_pos.rightdown);

    // 速度过小时，arctan分式计算误差大，无法准确反映方向，保持上次位置
    if (fabsf(steer_velt.leftup) < 5.f) steer_pos.leftup = steer_pos_pre.leftup;
    if (fabsf(steer_velt.leftdown) < 5.f) steer_pos.leftdown = steer_pos_pre.leftdown;
    if (fabsf(steer_velt.rightup) < 5.f) steer_pos.rightup = steer_pos_pre.rightup;
    if (fabsf(steer_velt.rightdown) < 5.f) steer_pos.rightdown = steer_pos_pre.rightdown;

    // 老代码的舵轮转向处理有问题，会让转角转到180°，实际转角至多为90°
    // // 处理转向跳变问题
    // if (fabsf(steer_pos.rightdown - steer_pos_pre.rightdown) > 90.f){
    //     steer_pos.rightdown = normalize_angle(steer_pos.rightdown + 180.f);
    //     steer_velt.rightdown = -steer_velt.rightdown;
    // }
    // if (fabsf(steer_pos.rightup - steer_pos_pre.rightup) > 90.f){
    //     steer_pos.rightup = normalize_angle(steer_pos.rightup + 180.f);
    //     steer_velt.rightup = -steer_velt.rightup;
    // }
    // if (fabsf(steer_pos.leftdown - steer_pos_pre.leftdown) > 90.f){
    //     steer_pos.leftdown = normalize_angle(steer_pos.leftdown + 180.f);
    //     steer_velt.leftdown = -steer_velt.leftdown;
    // }
    // if (fabsf(steer_pos.leftup - steer_pos_pre.leftup) > 90.f){
    //     steer_pos.leftup = normalize_angle(steer_pos.leftup + 180.f);
    //     steer_velt.leftup = -steer_velt.leftup;
    // }

    swerve_optimize(steer_pos_pre.rightdown, &steer_pos.rightdown, &steer_velt.rightdown);
    swerve_optimize(steer_pos_pre.rightup, &steer_pos.rightup, &steer_velt.rightup);
    swerve_optimize(steer_pos_pre.leftdown, &steer_pos.leftdown, &steer_velt.leftdown);
    swerve_optimize(steer_pos_pre.leftup, &steer_pos.leftup, &steer_velt.leftup);

    steer_pos_pre.leftup = steer_pos.leftup;
    steer_pos_pre.leftdown = steer_pos.leftdown;
    steer_pos_pre.rightup = steer_pos.rightup;
    steer_pos_pre.rightdown = steer_pos.rightdown;

    chassis_run.leftup.fpDes = steer_velt.leftup;
    chassis_run.leftdown.fpDes = steer_velt.leftdown;
    chassis_run.rightdown.fpDes = steer_velt.rightdown;
    chassis_run.rightup.fpDes = steer_velt.rightup;

    // +机械零点
    leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO + leftup_init_angle;
    leftdown_turn_motor.Input = steer_pos.leftdown * TURN_GEAR_RATIO + leftdown_init_angle;
    rightdown_turn_motor.Input = steer_pos.rightdown * TURN_GEAR_RATIO + rightdown_init_angle;
    rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO + rightup_init_angle ;
}

void Drive_Chassis(){
    if (nav.nav_state != CHASSIS_OFF)
    {
        PID_Calc(&chassis_run.rightdown, chassis_run.rightdown.fpDes, chassis_run.rightdown.fpFB);
        PID_Calc(&chassis_run.rightup, chassis_run.rightup.fpDes, chassis_run.rightup.fpFB);
        PID_Calc(&chassis_run.leftdown, chassis_run.leftdown.fpDes, chassis_run.leftdown.fpFB);
        PID_Calc(&chassis_run.leftup, chassis_run.leftup.fpDes, chassis_run.leftup.fpFB);

        DJI_ControlLoop(&rightdown_turn_motor);
        DJI_ControlLoop(&rightup_turn_motor);
        DJI_ControlLoop(&leftdown_turn_motor);
        DJI_ControlLoop(&leftup_turn_motor);
    }

    // 提高启动时的加速度，缩短小脚上台阶的加速距离
if (chassis_run.feed_forward_state == WITH_FORWARD){
        friction_compensation();
        cur_steer_velt.leftdown = ClipFloat(chassis_run.leftdown.fpU + friction_feedforward.leftdown, -16000.f, 16000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run.leftup.fpU + friction_feedforward.leftup, -16000.f, 16000.f);
        cur_steer_velt.rightdown = ClipFloat(chassis_run.rightdown.fpU + friction_feedforward.rightdown, -16000.f, 16000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run.rightup.fpU + friction_feedforward.rightup, -16000.f, 16000.f);
    }else{
        cur_steer_velt.leftdown = ClipFloat(chassis_run.leftdown.fpU, -16000.f, 16000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run.leftup.fpU, -16000.f, 16000.f);
        cur_steer_velt.rightdown = ClipFloat(chassis_run.rightdown.fpU, -16000.f, 16000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run.rightup.fpU, -16000.f, 16000.f);
    }
		
   CAN_SendCurrent(&hcan1, 0X200, leftdown_turn_motor.motor_current, cur_steer_velt.leftdown, rightdown_turn_motor.motor_current, cur_steer_velt.rightdown);
   CAN_SendCurrent(&hcan2, 0x200, leftup_turn_motor.motor_current, cur_steer_velt.leftup, rightup_turn_motor.motor_current, cur_steer_velt.rightup);
}

void friction_compensation(){
    friction_feedforward.leftup = ClipFloat(20.f * steer_velt.leftup, -1500.f, 1500.f); 
    friction_feedforward.leftdown = ClipFloat(20.f * steer_velt.leftdown, -1500.f, 1500.f); 
    friction_feedforward.rightup = ClipFloat(20.f * steer_velt.rightup, -1500.f, 1500.f); 
    friction_feedforward.rightdown = ClipFloat(20.f * steer_velt.rightdown, -1500.f, 1500.f);
}

// 返回相对ref最近的target等效角（可能超出[-180,180]，用于连续命令）
static float angle_nearest_to(float target_deg, float ref_deg)
{
    float delta = fmodf(target_deg - ref_deg, 360.0f);
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return ref_deg + delta;
}

// 舵轮转向优化，保证每次转向电机的转角至多为90°
static void swerve_optimize(float prev_deg, float *target_deg, float *wheel_vel)
{
    float cand = angle_nearest_to(*target_deg, prev_deg);
    float delta = cand - prev_deg;
    if (fabsf(delta) > 90.0f){
        cand = angle_nearest_to(*target_deg + 180.0f, prev_deg);
        *wheel_vel = -*wheel_vel;
    }
    *target_deg = cand; // 可能超出[-180,180]，但保证与prev的数值差≤90°
}
