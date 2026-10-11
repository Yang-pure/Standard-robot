#pragma once

#include <cmath>
#include <cstdint>

// 视觉目标和 IMU 当前角都用度；Yaw 电机目标用编码器值，Pitch 电机目标用弧度。
// 每轮只把一小部分角度误差加到上轮位置目标；进入死区后保持目标，避免反复摆动。
inline bool VisionAim(float targetYawDeg, float targetPitchDeg, float currentYawDeg, float currentPitchDeg, float yawEncoder, float pitchPosition, float& yawTarget, float& pitchTarget)
{
    if (!std::isfinite(targetYawDeg) || !std::isfinite(targetPitchDeg) || !std::isfinite(currentYawDeg) || !std::isfinite(currentPitchDeg) || !std::isfinite(yawEncoder) || !std::isfinite(pitchPosition))
    {
        return false;
    }

    float yawError = std::remainder(targetYawDeg - currentYawDeg, 360.0f);
    if (std::fabs(yawError) > 0.03f)  // 死区缩小到 0.03°
    {
        // 根据实际场景优化步进系数
        // 场景：5m 打 10cm 装甲板，半宽对应约 0.57°；为机械散布留出误差余量。
        float yawStep;
        if (std::fabs(yawError) > 1.2f)  // 跨装甲板切换（>2.4°/2）
        {
            yawStep = 0.13f;  // 13% 快速切换
        }
        else if (std::fabs(yawError) > 0.5f)  // 接近目标装甲板
        {
            yawStep = 0.08f;  // 8% 平稳接近
        }
        else if (std::fabs(yawError) > 0.2f)  // 装甲板内粗调
        {
            yawStep = 0.03f;  // 3% 精细调整
        }
        else  // 装甲板内精调（<0.2°）
        {
            yawStep = 0.01f;  // 1% 极致精度
        }

        // 单轮误差限为 ±2.5°，位置目标相对编码器的领先量另限为 ±10°。
        if (yawError > 2.5f)
        {
            yawError = 2.5f;
        }
        if (yawError < -2.5f)
        {
            yawError = -2.5f;
        }
        yawTarget = yawEncoder + std::remainder(yawTarget - yawEncoder, 8192.0f);
        yawTarget += yawError * (8192.0f / 360.0f) * yawStep;
        if (yawTarget > yawEncoder + 10.0f * 8192.0f / 360.0f)
        {
            yawTarget = yawEncoder + 10.0f * 8192.0f / 360.0f;
        }
        if (yawTarget < yawEncoder - 10.0f * 8192.0f / 360.0f)
        {
            yawTarget = yawEncoder - 10.0f * 8192.0f / 360.0f;
        }
    }

    float pitchError = targetPitchDeg - currentPitchDeg;
    if (std::fabs(pitchError) > 0.03f)  // 死区缩小到 0.03°
    {
        // Pitch 步进区间优化（误差范围更小，需要更精细的控制）
        float pitchStep;
        if (std::fabs(pitchError) > 3.0f)  // 大误差（>3°）
        {
            pitchStep = 0.05f;  // 5% 快速接近
        }
        else if (std::fabs(pitchError) > 1.0f)  // 大误差（>1°）
        {
            pitchStep = 0.03f;  // 3% 快速接近
        }
        else if (std::fabs(pitchError) > 0.5f)  // 中误差（0.5°-1°，接近允许范围）
        {
            pitchStep = 0.01f;  // 1% 平稳过渡
        }
        else if (std::fabs(pitchError) > 0.2f)  // 小误差（0.2°-0.5°，装甲板内调整）
        {
            pitchStep = 0.007f;  // 0.7% 精细调整
        }
        else  // 极小误差（<0.2°）
        {
            pitchStep = 0.004f;  // 0.4% 极致精度
        }

        // 限幅保护
        if (pitchError > 5.0f)
        {
            pitchError = 5.0f;
        }
        if (pitchError < -5.0f)
        {
            pitchError = -5.0f;
        }
        pitchTarget += pitchError * (3.1415926f / 180.0f) * pitchStep;
        if (pitchTarget > pitchPosition + 12.0f * 3.1415926f / 180.0f)
        {
            pitchTarget = pitchPosition + 12.0f * 3.1415926f / 180.0f;
        }
        if (pitchTarget < pitchPosition - 12.0f * 3.1415926f / 180.0f)
        {
            pitchTarget = pitchPosition - 12.0f * 3.1415926f / 180.0f;
        }
    }
    if (pitchTarget > 0.5f)
    {
        pitchTarget = 0.5f;
    }
    if (pitchTarget < -0.5f)
    {
        pitchTarget = -0.5f;
    }
    return true;
}

struct VisionShotAction
{
    bool flywheel;
    bool feed;
    bool single;
    bool abortSingle;
};

class VisionShot
{
public:
    void Reset()
    {
        armed = false;
        singleActive = false;
        seenBusy = false;
        hasNonzero = false;
		fault = false;
    }

    VisionShotAction Update(uint8_t shoot, bool valid, bool frictionReady, bool singleBusy, uint32_t nowMs)
    {
        if (!valid || shoot > 2)
        {
            Reset();
            return {false, false, false, false};
        }

		bool abortSingle = false;
        if (singleActive)
        {
            if (singleBusy)
            {
                seenBusy = true;
            }
            if (seenBusy && !singleBusy)
            {
                singleActive = false;
            }
			else if (nowMs - firedMs >= 2000u)
			{
				singleActive = false;
				fault = true;
				abortSingle = true;
			}
        }
		if (fault)
		{
			if (shoot == 0)
			{
				fault = false;
				armed = true;
				hasNonzero = false;
			}
			return {false, false, false, abortSingle};
		}

        if (shoot != 0)
        {
            lastNonzeroMs = nowMs;
            hasNonzero = true;
        }

        bool trigger = false;
        if (shoot == 0 && !singleActive)
        {
            armed = true;
        }
        if (shoot == 2 && armed && !singleActive && frictionReady)
        {
            trigger = true;
            armed = false;
            singleActive = true;
            seenBusy = false;
			firedMs = nowMs;
        }

        const bool flywheel = shoot != 0 || singleActive || (hasNonzero && nowMs - lastNonzeroMs < 500u);
        const bool feed = shoot == 1 && frictionReady && !singleActive;
        return {flywheel, feed, trigger, abortSingle};
    }

private:
    bool armed = false;
    bool singleActive = false;
    bool seenBusy = false;
    bool hasNonzero = false;
	bool fault = false;
    uint32_t lastNonzeroMs = 0;
	uint32_t firedMs = 0;
};
