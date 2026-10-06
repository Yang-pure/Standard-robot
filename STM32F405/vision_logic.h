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
    if (std::fabs(yawError) > 0.25f)
    {
        if (yawError > 10.0f)
        {
            yawError = 10.0f;
        }
        if (yawError < -10.0f)
        {
            yawError = -10.0f;
        }
        yawTarget = yawEncoder + std::remainder(yawTarget - yawEncoder, 8192.0f);
        yawTarget += yawError * (8192.0f / 360.0f) * 0.05f;
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
    if (std::fabs(pitchError) > 0.25f) // 到达目标附近后保持当前电机目标，不再追逐 IMU 的细小波动。
    {
        if (pitchError > 12.0f)
        {
            pitchError = 12.0f;
        }
        if (pitchError < -12.0f)
        {
            pitchError = -12.0f;
        }
        pitchTarget += pitchError * (3.1415926f / 180.0f) * 0.02f; // 小步推进位置目标，避免反馈延迟下反复越过目标。
        if (pitchTarget > pitchPosition + 12.0f * 3.1415926f / 180.0f)
        {
            pitchTarget = pitchPosition + 12.0f * 3.1415926f / 180.0f;
        }
        if (pitchTarget < pitchPosition - 12.0f * 3.1415926f / 180.0f)
        {
            pitchTarget = pitchPosition - 12.0f * 3.1415926f / 180.0f;
        }
    }
    if (pitchTarget > 1.0f)
    {
        pitchTarget = 1.0f;
    }
    if (pitchTarget < -1.0f)
    {
        pitchTarget = -1.0f;
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
