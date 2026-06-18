/*******************************************************************************
 * @file        VOFA+的通信协议
 * @author      eternal_fu
 * @version     V0.0.1
 * @brief       
 * @Date 2024-12-11 01:05:50
 * @LastEditTime 2024-12-11 04:28:17
 * @**************************************************************************** 
 * @attention   修改说明
 * @    版本号      日期                作者      说明
 *      0.0.1       2024年12月11日      fu      仅支持FireWater协议格式、协议处理
 *******************************************************************************/
#include "uart_vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // 字符串处理相关头文件
#include "PID.h"
#include "Serial.h"

extern PID_t Motor_Left;
extern PID_t Motor_Right;

//#include "pid_demo.h"

/* VOFA数据缓存数组 */
uint8_t g_ucVofaBuf[VOFA_DATAPACK_MAXLEN];

static float vofa_get_data(uint8_t, uint8_t);
static void vofa_set_sram_data(uint8_t, float);
//static void vofa_set_flash_data(float);

/* 函数实现 */

/*******************************************************************************
 * @brief   向VOFA+发送PID实时数据（FireWater协议）
 * @param   _dataflag: 0-左轮PID  1-右轮PID  2-同时发送左右轮
 *******************************************************************************/
void vofa_draw_graphical(uint8_t _dataflag)
{
    switch(_dataflag)
    {
        // 发送左轮PID数据：目标值,实际值,输出,Kp,Ki,Kd
        case 0:
            Serial_Printf("LeftPID:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        Motor_Left.Target,
                        Motor_Left.Actual,
                        Motor_Left.Out,
                        Motor_Left.Kp,
                        Motor_Left.Ki,
                        Motor_Left.Kd);
            break;
            
        // 发送右轮PID数据：目标值,实际值,输出,Kp,Ki,Kd
        case 1:
            Serial_Printf("RightPID:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        Motor_Right.Target,
                        Motor_Right.Actual,
                        Motor_Right.Out,
                        Motor_Right.Kp,
                        Motor_Right.Ki,
                        Motor_Right.Kd);
            break;
            
        // 同时发送左右轮（VOFA+可拆分多通道显示）
        case 2:
            Serial_Printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        // 左轮6个参数
                        Motor_Left.Target, Motor_Left.Actual, Motor_Left.Out,
                        Motor_Left.Kp, Motor_Left.Ki, Motor_Left.Kd,
                        // 右轮6个参数
                        Motor_Right.Target, Motor_Right.Actual, Motor_Right.Out,
                        Motor_Right.Kp, Motor_Right.Ki, Motor_Right.Kd);
            break;
            
        default:
            break;
    }
}
#if 0 /* 参考示例(可删除) */
    switch(_dataflag)
    {
        case _left_motor_pid:
            //左轮速度环参数目标和当前值
            VOFA_printf("%.2f,"		
                        "%.2f,"
                        "%.2f,"
                        "%.2f\r\n"
                        ,left_Motor.pid.target, left_Motor.pid.current, left_Motor.pid.kp, left_Motor.pid.ki);
            break;
        case _right_motor_pid:
            //右轮速度环参数目标和当前值
            VOFA_printf("%f,"
                        "%f,"
                        "%f,"
                        "%f\r\n"
                        ,right_Motor.pid.target, right_Motor.pid.current, right_Motor.pid.kp, right_Motor.pid.ki);
            break;
        case _turn_pid:
            //转向pid(车头)
            VOFA_printf("%.2f,"		
                        "%.2f,"
                        "%.2f\r\n"
                        ,turn_pid.target, turn_pid.current, turn_pid.kp);
            break;
        case _angle_pid:
            //角度转向环
            VOFA_printf("%.0f,"		
                        "%.0f,"
                        "%.2f,"
                        "%.0f\r\n"
                        ,angle_pid.target, angle_pid.current, angle_pid.kp, angle_pid.kd);
            break;
        case _track_pid:
            //巡线，舵机，转向环
            VOFA_printf("%.0f,"		
                        "%.0f,"
                        "%.5f,"
                        "%.0f\r\n"
                        ,track_pid.target, track_pid.current, track_pid.kp, track_pid.kd);
            break;
        case _dis_pid:
            //位移参数
            VOFA_printf("%.0f,"		
                        "%.0f,"
                        "%.4f,"
                        "%.4f,"
                        "%.4f\r\n"
                        ,dis_pid.target, dis_pid.current, dis_pid.kp, dis_pid.kd,  dis_pid.out);
            break;
    }
#endif

void vofa_set_data(uint8_t _rx_byte)
{
    static uint8_t end_pos = 0;         
    static uint8_t head_pos = 0;        
    static uint8_t wait_n = 0; // 新增：标记是否收到\r，等待\n

    // 写入字节到缓存
    g_ucVofaBuf[end_pos] = _rx_byte;

    // 第一步：找帧头@
    if (_rx_byte == '@') { // 只改这里：帧头从=改成@
        head_pos = end_pos;
        wait_n = 0; // 重置等待\n的标记
    }
    // 第二步：收到\r，标记要等\n
    else if (_rx_byte == '\r') {
        wait_n = 1;
    }
    // 第三步：收到\n，且之前有@和\r，才解析
    else if (_rx_byte == '\n' && wait_n == 1 && g_ucVofaBuf[head_pos] == '@') {
        // 解析数值（跳过@，找=的位置）
        float param_val = vofa_get_data(head_pos, end_pos);
        vofa_set_sram_data(head_pos, param_val);

        // 清空缓存
        end_pos = 0;
        head_pos = 0;
        wait_n = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }

    // 防止越界
    if(++end_pos >= VOFA_DATAPACK_MAXLEN) {
        end_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }
}
static void vofa_set_sram_data(uint8_t _head, float _data)
{
    // 跳过@，找指令标识（如P2/I2/T2）
    uint8_t cmd_pos = _head + 1; // @的下一位是指令开头（如P2的P）
    
    // 检查是否有足够长度（至少2个字符：P2、I2等）
    if (cmd_pos + 1 >= VOFA_DATAPACK_MAXLEN) {
        Serial_Printf("VOFA Cmd Too Short\r\n");
        return;
    }

    // 提取指令（如P2、I2、T2）
    uint8_t cmd1 = g_ucVofaBuf[cmd_pos];   // P/I/T
    uint8_t cmd2 = g_ucVofaBuf[cmd_pos+1]; // 2

    // 只处理速度环P2/I2/T2（你的需求）
    if (cmd2 == '2') {
        if (cmd1 == 'P') {
            Motor_Left.Kp = _data;
            Serial_Printf("Set LeftKp=%.2f\r\n", _data);
        } else if (cmd1 == 'I') {
            Motor_Left.Ki = _data;
            Serial_Printf("Set LeftKi=%.3f\r\n", _data);
        } else if (cmd1 == 'T') {
            Motor_Left.Target = _data;
            Serial_Printf("Set LeftTarget=%.2f\r\n", _data);
        }
    } else {
        Serial_Printf("Unknown Cmd: %c%c\r\n", cmd1, cmd2);
    }
}
static float vofa_get_data(uint8_t head, uint8_t end)
{
    char val_str[VOFA_DATAPACK_MAXLEN] = {0};
    uint8_t val_idx = 0;

    // 从@的下一位开始，找到=后，取后面的数值
    for (uint8_t i = head + 1; i < end; i++) {
        if (g_ucVofaBuf[i] == '=') { // 找到=，开始取数值
            i++; // 跳过=
            while (i < end && g_ucVofaBuf[i] != '\r' && g_ucVofaBuf[i] != '\n') {
                val_str[val_idx++] = g_ucVofaBuf[i];
                i++;
            }
            break;
        }
    }

    return atof(val_str); // 转浮点
}

void vofa_parse_packet(char *packet)
{
    char cmd1;
    char cmd2;
    float value;

    // 解析格式：P2=1.23
    if (sscanf(packet, "@%c%c=%f", &cmd1, &cmd2, &value) != 3)
    {
        Serial_Printf("VOFA Parse Error: %s\r\n", packet);
        return;
    }

    if (cmd2 == '2')   // 左轮 PID
    {
        if (cmd1 == 'P')
        {
            Motor_Left.Kp = value;
            Serial_Printf("Set Left Kp = %.3f\r\n", value);
        }
        else if (cmd1 == 'I')
        {
            Motor_Left.Ki = value;
            Serial_Printf("Set Left Ki = %.3f\r\n", value);
        }
        else if (cmd1 == 'D')
        {
            Motor_Left.Kd = value;
            Serial_Printf("Set Left Kd = %.3f\r\n", value);
        }
        else if (cmd1 == 'T')
        {
            Motor_Left.Target = value;
            Serial_Printf("Set Left Target = %.3f\r\n", value);
        }
    }
    else if (cmd2 == '3')   // 右轮 PID
    {
        if (cmd1 == 'P')
        {
            Motor_Right.Kp = value;
            Serial_Printf("Set Right Kp = %.3f\r\n", value);
        }
        else if (cmd1 == 'I')
        {
            Motor_Right.Ki = value;
            Serial_Printf("Set Right Ki = %.3f\r\n", value);
        }
        else if (cmd1 == 'D')
        {
            Motor_Right.Kd = value;
            Serial_Printf("Set Right Kd = %.3f\r\n", value);
        }
        else if (cmd1 == 'T')
        {
            Motor_Right.Target = value;
            Serial_Printf("Set Right Target = %.3f\r\n", value);
        }
    }
    else
    {
        Serial_Printf("Unknown PID ID: %c%c\r\n", cmd1, cmd2);
    }
}


///*******************************************************************************
// * @brief   接收并解析上位机下发的浮点参数(半解析半判断处理)
// *          当接收到VOFA+下发的指令时，执行对应处理
// * @return {*}
// * @note    目前仅实现VOFA参数的接收
// *******************************************************************************/
//void vofa_set_data(uint8_t _rx_byte)
//{
//    static uint8_t end_pos = 0;         // 记录vofa缓存数组的结尾位置，当前位置
//    static uint8_t head_pos = 0;

//    /* 数据写入 */
//    g_ucVofaBuf[end_pos] = _rx_byte;

//    if (_rx_byte == VOFA_DATAPACK_HEAD) {   /* 找到帧头 */
//        head_pos = end_pos;                 /* 记录当前位置为帧头 */
//    }
//    else if(_rx_byte == VOFA_DATAPACK_END && g_ucVofaBuf[head_pos] == VOFA_DATAPACK_HEAD) { /* 找到帧尾且帧头有效时 */
//        /* 解析VOFA数据 */
//        float VofaData = vofa_get_data(head_pos, end_pos);
//        
//        /* 解析完成，写入存储在SRAM的参数 */
//        vofa_set_sram_data(head_pos, VofaData);

//        /* 清空缓存 */
//        end_pos = 0;
//        head_pos = 0;
//        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
//    }

//    /* 防止数组越界 */
//    if(++end_pos > VOFA_DATAPACK_MAXLEN) {
//        end_pos = 0;
//        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
//    }
//}


///*******************************************************************************
// * @brief   解析VOFA下发的浮点参数
// * @return {*}
// * @note    _head是帧头位置，_end是帧尾位置，数据起始为_head+1
// *          c标准库函数, 需要头文件 stdlib.h
// *          - atof()：字符串转浮点型
// *          - atoi()：字符串转整型
// *******************************************************************************/
//static float vofa_get_data(uint8_t _head, uint8_t _end)
//{
//    char _VofaData[_end - (_head + 1)];
//    for(uint8_t i = 0; i < _end - _head - 1; i++)  // 修复数组越界问题
//        _VofaData[i] = g_ucVofaBuf[i + _head + 1]; 

//    return atof(_VofaData); /* 字符串转浮点 */
//}

///*******************************************************************************
// * @brief   将接收的参数，直接写入SRAM区域，掉电丢失
// *          在此处将接收到的值修改对应参数
// * @return {*}
// * @note    在此处扩展指令集
// * 数据包格式说明：
// * |   参数标识(用于区分下发的是什么参数,1-2byte)   |   帧头('=',1byte)   |   浮点值(n byte)   |   帧尾('\n',1byte)   |
// * 
// *  示例:
// *      "P1=%.2f\n"
// *      "V1=%.2f\n"
// * 
// *  其中P1/V1是参数标识，约定规则：
// *  P1 - car_pid_kp
// *  P2 - turn_pid_kp
// *  I1 - car_pid_ki
// *  I2=%.2f\n
// *  可按此规则扩展
// * 
// *******************************************************************************/
//static void vofa_set_sram_data(uint8_t _head, float _data)
//{
//    uint8_t _id_pos1 = _head - 2;   /* 参数标识的第1位 - P/I/... */
//    uint8_t _id_pos2 = _head - 1;   /* 参数标识的第2位 - 1/2/3/4/...*/
////	
////    // 角度环[1] 
////    /* P1 - g_tAnglePID.kp */
////    if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '1') { 
////        g_tAnglePID.kp = _data;
////    }
////    /* D1 - g_tAnglePID.kd */
////    else if (g_ucVofaBuf[_id_pos1] == 'D' && g_ucVofaBuf[_id_pos2] == '1') {
////        g_tAnglePID.kd = _data;
////    }
////    /* T1 - g_tAnglePID.target */
////    else if (g_ucVofaBuf[_id_pos1] == 'T' && g_ucVofaBuf[_id_pos2] == '1') {
////        g_tAnglePID.target = _data;
////    }
//    // 速度环[2] 
//    /* P2 - g_tLeftMotorPID.kp */
//     if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '2') { 
//        Motor_Left.Kp = _data;
//    }
//    /* I2 - g_tLeftMotorPID.ki */
//    else if (g_ucVofaBuf[_id_pos1] == 'I' && g_ucVofaBuf[_id_pos2] == '2') {
//        Motor_Left.Ki = _data;
//    }
//    /* T2 - g_tLeftMotorPID.target */
//    else if (g_ucVofaBuf[_id_pos1] == 'T' && g_ucVofaBuf[_id_pos2] == '2') {
//        Motor_Left.Kd = _data;
//    }

//    /* 扩展示例 */
//    /* 注意如果是int类型参数-需要强转(int) */
//}

/*******************************************************************************
 * @brief   将接收的参数，直接写入FLASH区域，掉电不丢失
 *          目前FLASH区域未规划存储区域，未实现FLASH操作API
 * @return {*}
 * @note    none
 *******************************************************************************/
//static void vofa_set_flash_data(float _data)
//{
//    
//}