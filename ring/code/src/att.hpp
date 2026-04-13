#include <math.h>

#define DCM_KP_MAG 10.000f // 磁力计补偿陀螺仪PI参数1.00
#define DCM_KI_MAG 0.000f // 0

#define DCM_KP_ACC 0.600f // 加速度补偿陀螺仪PI参数  0.600
#define DCM_KI_ACC 0.005f // 0.005

#define PI 3.1415926535

#define SPIN_RATE_LIMIT 20 // 旋转速率
#define RAD (PI / 180.0f)
#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)

class att
{
private:
    /* data */
    float rMat[3][3];                                 // 四元数的旋转矩阵
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数

    void imuComputeRotationMatrix(void)
    {
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q3 = q2 * q3;

        rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
        rMat[0][1] = 2.0f * (q1q2 + -q0q3);
        rMat[0][2] = 2.0f * (q1q3 - -q0q2);

        rMat[1][0] = 2.0f * (q1q2 - -q0q3);
        rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
        rMat[1][2] = 2.0f * (q2q3 + -q0q1);

        rMat[2][0] = 2.0f * (q1q3 + -q0q2);
        rMat[2][1] = 2.0f * (q2q3 - -q0q1);
        rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
    }
    float imuMagFastPGainSaleFactor(void)
    {
        // 四轴上电后磁力计融合需要一段时间
        // 为了快速融合，前100次使用快速增益
        static uint32_t magFastPGainCount = 100;

        if (magFastPGainCount)
        {
            magFastPGainCount--;
            return 50.0f;
        }
        else
        {
            return 1.0f;
        }
    }

    float invSqrt(float x)
    {
        return 1.0f / sqrtf(x);
    }

    float imuAcceFastPGainSaleFactor(void)
    {
        // 四轴上电后加速度计融合需要一段时间
        // 为了快速融合，前20次使用快速增益
        static uint32_t acceFastPGainCount = 100;

        if (acceFastPGainCount)
        {
            acceFastPGainCount--;
            return 50.0f;
        }
        else
        {
            return 1.0f;
        }
    }

    float MyAtan2Approx(float y, float x)
    {
#define atanPolyCoef1 3.14551665884836e-07f
#define atanPolyCoef2 0.99997356613987f
#define atanPolyCoef3 0.14744007058297684f
#define atanPolyCoef4 0.3099814292351353f
#define atanPolyCoef5 0.05030176425872175f
#define atanPolyCoef6 0.1471039133652469f
#define atanPolyCoef7 0.6444640676891548f

        float res, absX, absY;
        absX = fabsf(x);
        absY = fabsf(y);
        res = max(absX, absY);
        if (res)
            res = min(absX, absY) / res;
        else
            res = 0.0f;
        res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
        if (absY > absX)
            res = (PI / 2.0f) - res;
        if (x < 0)
            res = PI - res;
        if (y < 0)
            res = -res;
        return res;
    }

    void imuUpdateEulerAngles(void)
    {
        // NED
        Pitch = RADIANS_TO_DEGREES(atan2(rMat[2][1], rMat[2][2])); //+-180
        Roll = RADIANS_TO_DEGREES(asinf(-rMat[2][0]));             //+-90
        Yaw = RADIANS_TO_DEGREES(atan2(rMat[1][0], rMat[0][0]));

        Pitch += 180;
        if (Pitch > 180)
            Pitch -= 360;

        Yaw = -Yaw;
        Yaw += 180;
    }

public:
    att(/* args */);
    ~att();
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                bool useMag, float dt);
    float Pitch, Roll, Yaw;
};

att::att(/* args */)
{
}

att::~att()
{
}

void att::update(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float mx, float my, float mz,
                 bool useMag, float dt)
{
    static float integralAccX = 0.0f, integralAccY = 0.0f, integralAccZ = 0.0f; // 加速度积分误差
    static float integralMagX = 0.0f, integralMagY = 0.0f, integralMagZ = 0.0f; // 磁力计积分误差
    float ex, ey, ez;

    // 计算旋转速率(rad/s)
    const float spin_rate_sq = sqrt(gx) + sqrt(gy) + sqrt(gz);

    // Step 1: Yaw correction
    if (useMag)
    {
        const float magMagnitudeSq = mx * mx + my * my + mz * mz;
        float kpMag = DCM_KP_MAG * imuMagFastPGainSaleFactor();

        if (magMagnitudeSq > 0.01f)
        {
            // 单位化磁力计测量值
            const float magRecipNorm = invSqrt(magMagnitudeSq);
            mx *= magRecipNorm;
            my *= magRecipNorm;
            mz *= magRecipNorm;

            // 计算X\Y方向的磁通，磁北方向磁通
            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            const float bx = sqrtf(hx * hx + hy * hy);

            // 磁力计误差是估计磁北和测量磁北之间的交叉乘积
            const float ez_ef = -(hy * bx);

            // 旋转误差到机体坐标系
            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else
        {
            ex = 0;
            ey = 0;
            ez = 0;
        }

        // 累计误差补偿
        if (DCM_KI_MAG > 0.0f)
        {
            // 如果旋转速率大于限制值则停止积分
            if (spin_rate_sq < sqrt(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
            {
                integralMagX += DCM_KI_MAG * ex * dt;
                integralMagY += DCM_KI_MAG * ey * dt;
                integralMagZ += DCM_KI_MAG * ez * dt;

                gx += integralMagX;
                gy += integralMagY;
                gz += integralMagZ;
            }
        }

        // 误差补偿
        gx += kpMag * ex;
        gy += kpMag * ey;
        gz += kpMag * ez;
    }

    // Step 2: Roll and pitch correction
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 单位化加速计测量值
        const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);
        float KpAcce = DCM_KP_ACC * imuAcceFastPGainSaleFactor();

        ax *= accRecipNorm;
        ay *= accRecipNorm;
        az *= accRecipNorm;

        // 加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
        ex = (ay * rMat[2][2] - az * rMat[2][1]);
        ey = (az * rMat[2][0] - ax * rMat[2][2]);
        ez = (ax * rMat[2][1] - ay * rMat[2][0]);

        // 累计误差补偿
        if (DCM_KI_ACC > 0.0f)
        {
            // 如果旋转速率大于限制值则停止积分
            if (spin_rate_sq < sqrt(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
            {
                integralAccX += DCM_KI_ACC * ex * dt;
                integralAccY += DCM_KI_ACC * ey * dt;
                integralAccZ += DCM_KI_ACC * ez * dt;

                gx += integralAccX;
                gy += integralAccY;
                gz += integralAccZ;
            }
        }

        // 误差补偿
        gx += KpAcce * ex;
        gy += KpAcce * ey;
        gz += KpAcce * ez;
    }

    // 一阶近似算法，四元数运动学方程的离散化形式和积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 单位化四元数
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    // 计算四元数的旋转矩阵
    imuComputeRotationMatrix();

    imuUpdateEulerAngles();
}