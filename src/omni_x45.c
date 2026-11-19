#include "omni_x45.h"
#include <math.h>
#include <stddef.h>

static bool positive_finite(float value)
{
    return isfinite(value) && value > 0.0f;
}

bool omni_x45_rpm(const OmniX45Config *cfg, float vx, float vy, float omega_rad_s, float yaw_rad,
                  float rpm[4])
{
    if (rpm == NULL)
        return false;
    for (int i = 0; i < 4; ++i)
        rpm[i] = 0.0f;
    if (cfg == NULL || !isfinite(vx) || !isfinite(vy) || !isfinite(omega_rad_s) ||
        !isfinite(yaw_rad) || !positive_finite(cfg->wheel_radius_m) ||
        !positive_finite(cfg->half_length_m) || !positive_finite(cfg->half_width_m) ||
        !positive_finite(cfg->max_motor_rpm) || !positive_finite(cfg->motor_turns_per_wheel_turn))
        return false;

    /* 先把场地速度绕 -yaw 转到车体系。角速度在这两个平面坐标系中相同。 */
    const float co = cosf(yaw_rad), si = sinf(yaw_rad);
    const float x = co * vx + si * vy;
    const float y = -si * vx + co * vy;
    const float spin = -(cfg->half_length_m + cfg->half_width_m) * omega_rad_s;
    /* 实车公共轮速为负时整车逆时针，因此旋转项取负号。
     * 平移项由前进/左移实测保留；1/√2 是 45° 方向投影。 */
    const float projected[4] = {-x - y + spin, -x + y + spin, x + y + spin, x - y + spin};
    const float to_rpm =
        0.7071067812f * 9.549296586f * cfg->motor_turns_per_wheel_turn / cfg->wheel_radius_m;
    float target[4], largest = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        if (cfg->sign[i] != 1 && cfg->sign[i] != -1)
            return false;
        target[i] = projected[i] * to_rpm * (float)cfg->sign[i];
        if (!isfinite(target[i]))
            return false;
        largest = fmaxf(largest, fabsf(target[i]));
    }
    const float scale = largest > cfg->max_motor_rpm ? cfg->max_motor_rpm / largest : 1.0f;
    for (int i = 0; i < 4; ++i)
        rpm[i] = target[i] * scale;
    return true;
}
