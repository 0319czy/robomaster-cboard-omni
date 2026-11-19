#ifndef OMNI_X45_H
#define OMNI_X45_H
#include <stdbool.h>

/* 四个 ±45° 安装的全向轮。数组按 CAN ID 1 左后、2 左前、3 右前、4 右后排列。
 * sign 为电调正转对该轮切向正方向的符号，实测后填 +1 或 -1；未标定留 0。
 * 半长/半宽从旋转中心量到轮心。转速上限指电机转子，不是减速箱输出轴。
 */
typedef struct
{
    float half_length_m, half_width_m, wheel_radius_m;
    float motor_turns_per_wheel_turn, max_motor_rpm;
    int sign[4];
} OmniX45Config;

/* 平移按场地坐标输入 (m/s)，车头初始方向为 x，向左为 y。
 * yaw_rad 为车头相对初始朝向的逆时针角度；车体相对模式传 0。
 * omega_rad_s 正值逆时针，负值顺时针。目标按比例限速。
 * 参数或标定无效返回 false 并将输出清零。函数只计算，不访问外设。
 */
bool omni_x45_rpm(const OmniX45Config *cfg, float vx, float vy, float omega_rad_s, float yaw_rad,
                  float rpm[4]);
#endif
