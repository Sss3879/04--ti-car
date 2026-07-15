#include "filter.h"

/**
 * @brief 初始化滤波通道
 */
void Filter_Init(Filter_Channel *ch, float alpha,
                 int16_t dead_zone, int16_t max_delta)
{
    ch->raw        = 0;
    ch->ema        = 0;
    ch->output     = 0;
    ch->last_output = 0;
    ch->alpha      = alpha;
    ch->dead_zone  = dead_zone;
    ch->max_delta  = max_delta;
    ch->is_first   = 1;
}

/**
 * @brief 三步滤波处理流水线
 *
 *  串口原始值 → [死区] → [EMA低通] → [限幅] → 最终输出
 */
int16_t Filter_Process(Filter_Channel *ch, int16_t raw)
{
    int16_t after_dead;
    int16_t after_ema;
    int32_t delta;

    /* ===== 第1步：死区滤波 ===== */
    if ((raw < ch->dead_zone) && (raw > -ch->dead_zone)) {
        after_dead = 0;
    } else {
        after_dead = raw;
    }
    ch->raw = after_dead;

    /* ===== 第2步：一阶EMA低通滤波 ===== */
    if (ch->is_first) {
        /* 首帧直接用当前值，避免从0缓慢爬升 */
        after_ema = after_dead;
        ch->last_output = after_ema;  /* 同步基准值，跳过首帧限幅 */
        ch->is_first = 0;
    } else {
        /* ema = alpha * raw + (1 - alpha) * last_ema */
        after_ema = (int16_t)(ch->alpha * after_dead +
                              (1.0f - ch->alpha) * ch->ema);
    }
    ch->ema = after_ema;

    /* ===== 第3步：限幅滤波（Delta Clamp）===== */
    delta = (int32_t)after_ema - (int32_t)ch->last_output;
    if (delta > ch->max_delta) {
        after_ema = ch->last_output + ch->max_delta;
    } else if (delta < -ch->max_delta) {
        after_ema = ch->last_output - ch->max_delta;
    }
    ch->output      = after_ema;
    ch->last_output = after_ema;

    return ch->output;
}

/**
 * @brief 获取当前滤波输出
 */
int16_t Filter_GetOutput(Filter_Channel *ch)
{
    return ch->output;
}

/**
 * @brief 重置滤波状态
 */
void Filter_Reset(Filter_Channel *ch)
{
    ch->raw         = 0;
    ch->ema         = 0;
    ch->output      = 0;
    ch->last_output = 0;
    ch->is_first    = 1;
}
