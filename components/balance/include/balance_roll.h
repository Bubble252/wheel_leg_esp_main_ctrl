#pragma once

/**
 * @brief Roll 控制 + X-Offset 辅助函数
 *
 * 将 X-Offset 笛卡尔偏移 + 腿长限幅 + 工作空间限幅 + 逆运动学
 * 封装为可复用函数, 消灭 compute_balance_output() 中的 4 处重复代码.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 对左右腿分别应用 X-Offset + 腿长限幅 + 工作空间限幅 + IK
 * @param left_length   左腿腿长 (in/out)
 * @param left_angle    左腿身体夹角 (in/out)
 * @param right_length  右腿腿长 (in/out)
 * @param right_angle   右腿身体夹角 (in/out)
 * @param xoffset       X-Offset 偏移量 (米)
 * @note 会更新全局腿目标变量 (g_leg_left_target_*, g_leg_right_target_*)
 */
void apply_xoffset_and_ik(float *left_length, float *left_angle,
                           float *right_length, float *right_angle,
                           float xoffset);

/**
 * @brief 对双腿 (同参数) 应用 X-Offset + 限幅 + IK
 * @param base_length   基础腿长
 * @param base_angle    基础身体夹角
 * @param xoffset       X-Offset 偏移量 (米)
 * @note 会更新全局腿目标变量
 */
void apply_xoffset_single(float base_length, float base_angle, float xoffset);

#ifdef __cplusplus
}
#endif
