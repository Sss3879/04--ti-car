#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

/* ========== 滤波参数配置 ========== */

/* 死区阈值：|误差| < 此值时强制归零，防止中心附近振荡 */
#define FILTER_DEAD_ZONE_X   3
#define FILTER_DEAD_ZONE_Y   3

/* EMA 平滑系数 (0.0 ~ 1.0)
   越大响应越快但越抖，越小越平滑但越滞后
   0.15~0.25: 适合稳定归中
   0.30~0.50: 适合快速跟踪  */
#define FILTER_ALPHA_X       0.40f
#define FILTER_ALPHA_Y       0.40f

/* 限幅：相邻两次输出最大变化量，防止 K230 误检突变 */
#define FILTER_MAX_DELTA_X   30
#define FILTER_MAX_DELTA_Y   30


/* ========== 滤波通道数据结构 ========== */
typedef struct {
    int16_t raw;              /* 最新原始输入（死区处理后） */
    int16_t ema;              /* EMA 一阶低通滤波输出 */
    int16_t output;           /* 最终输出（限幅后） */
    int16_t last_output;      /* 上一帧最终输出（限幅用） */

    float   alpha;            /* EMA 平滑系数 */
    int16_t dead_zone;        /* 死区阈值 */
    int16_t max_delta;        /* 最大单帧变化量 */
    uint8_t is_first;         /* 首次采样标记 */
} Filter_Channel;


/* ========== 滤波函数 ========== */

/**
 * @brief 初始化滤波通道
 * @param ch    滤波通道指针
 * @param alpha EMA平滑系数 (0.0~1.0)
 * @param dead_zone 死区阈值
 * @param max_delta 最大单帧变化量
 */
void Filter_Init(Filter_Channel *ch, float alpha,
                 int16_t dead_zone, int16_t max_delta);

/**
 * @brief 三步滤波处理：
 *        1. 死区    — |raw| < dead_zone → 归零
 *        2. EMA 低通 — 一阶指数平滑
 *        3. 限幅    — 限制相邻帧最大变化量
 *
 * @param ch  滤波通道指针
 * @param raw 最新的原始测量值
 * @return    滤波后的最终输出值
 */
int16_t Filter_Process(Filter_Channel *ch, int16_t raw);

/**
 * @brief 获取当前滤波输出（不处理新数据）
 */
int16_t Filter_GetOutput(Filter_Channel *ch);

/**
 * @brief 重置滤波状态（目标丢失 / 重新跟踪时调用）
 */
void Filter_Reset(Filter_Channel *ch);

#endif /* __FILTER_H */
