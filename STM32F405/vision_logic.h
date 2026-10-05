#pragma once

#include <cmath>
#include <cstdint>

// 视觉角度是 IMU 零点下的绝对角（度）；电机目标由当前反馈计算，避免累加漂移。
inline bool VisionAim(float targetYawDeg, float targetPitchDeg, float currentYawDeg, float currentPitchDeg, float yawEncoder, float pitchPosition, float& yawTarget, float& pitchTarget)
{
    if (!std::isfinite(targetYawDeg) || !std::isfinite(targetPitchDeg) || !std::isfinite(currentYawDeg) || !std::isfinite(currentPitchDeg) || !std::isfinite(yawEncoder) || !std::isfinite(pitchPosition))
    {
        return false;
    }

    float yawError = std::remainder(targetYawDeg - currentYawDeg, 360.0f);
    if (yawError > 10.0f)
    {
        yawError = 10.0f;
    }
    if (yawError < -10.0f)
    {
        yawError = -10.0f;
    }
    yawTarget = yawEncoder + yawError * (8192.0f / 360.0f);

    float pitchError = targetPitchDeg - currentPitchDeg;
    if (pitchError > 12.0f)
    {
        pitchError = 12.0f;
    }
    if (pitchError < -12.0f)
    {
        pitchError = -12.0f;
    }
    pitchTarget = pitchPosition + pitchError * (3.1415926f / 180.0f);
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
