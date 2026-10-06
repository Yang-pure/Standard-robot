#include "vision_logic.h"
#include <cassert>
#include <cmath>

int main()
{
    float yaw = 1000.0f;
    float pitch = 0.0f;
    assert(VisionAim(179.0f, -20.0f, -179.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - (1000.0f - 0.1f * 8192.0f / 360.0f)) < 0.1f);
    assert(std::fabs(pitch + 0.24f * 3.1415926f / 180.0f) < 0.001f);
    yaw = 1000.0f;
    pitch = 0.0f;
    assert(VisionAim(10.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - (1000.0f + 0.5f * 8192.0f / 360.0f)) < 0.1f);
    pitch = 0.0f;
    assert(VisionAim(0.0f, 12.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(pitch - 0.24f * 3.1415926f / 180.0f) < 0.001f);
    pitch = 0.1f;
    assert(VisionAim(0.0f, 0.2f, 0.0f, 0.0f, 1000.0f, 0.1f, yaw, pitch));
    assert(std::fabs(pitch - 0.1f) < 0.00001f);
    assert(VisionAim(0.0f, -0.2f, 0.0f, 0.0f, 1000.0f, 0.1f, yaw, pitch));
    assert(std::fabs(pitch - 0.1f) < 0.00001f);
    assert(VisionAim(0.0f, 0.27f, 0.0f, 0.0f, 1000.0f, 0.1f, yaw, pitch));
    assert(pitch > 0.1f);
    pitch = 0.1f;
    assert(VisionAim(0.0f, 0.4f, 0.0f, 0.0f, 1000.0f, 0.1f, yaw, pitch));
    assert(pitch > 0.1f);
    yaw = 500.0f;
    pitch = 0.35f;
    assert(VisionAim(15.0f, 7.0f, 15.0f, 7.0f, 500.0f, 0.35f, yaw, pitch));
    assert(std::fabs(pitch - 0.35f) < 0.0001f);
    yaw = 1000.0f;
    pitch = 0.0f;
    assert(VisionAim(0.2f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - 1000.0f) < 0.001f);
    assert(VisionAim(-0.2f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - 1000.0f) < 0.001f);
    assert(VisionAim(0.2f, 12.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - 1000.0f) < 0.001f && pitch > 0.0f);
    assert(VisionAim(0.3f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw > 1000.0f);
    yaw = 1000.0f;
    assert(VisionAim(-0.3f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw < 1000.0f);
    assert(VisionAim(0.8f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw > 1000.0f);
    assert(VisionAim(1.2f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw > 1000.0f);
    yaw = 1000.0f;
    assert(VisionAim(1.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw > 1000.0f);
    yaw = 1000.0f;
    assert(VisionAim(-1.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw < 1000.0f);
    yaw = 1000.0f;
    pitch = 0.0f;
    assert(VisionAim(-1.0f, -1.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    assert(yaw < 1000.0f && pitch < 0.0f);
    assert(!VisionAim(NAN, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, yaw, pitch));
    pitch = -0.999f;
    assert(VisionAim(0.0f, -100.0f, 0.0f, 0.0f, 1000.0f, -0.79f, yaw, pitch));
    assert(std::fabs(pitch + 1.0f) < 0.001f);

    VisionShot shot;
    auto a = shot.Update(0, true, false, false, 0);
    assert(!a.flywheel && !a.feed && !a.single);
    a = shot.Update(2, true, false, false, 10);
    assert(a.flywheel && !a.single);
    a = shot.Update(2, true, true, false, 20);
    assert(a.single && !a.feed);
    a = shot.Update(2, true, true, true, 30);
    assert(!a.single);
    a = shot.Update(2, true, true, false, 40);
    assert(!a.single);
    a = shot.Update(0, true, false, false, 50);
    assert(a.flywheel);
    a = shot.Update(2, true, true, false, 60);
    assert(a.single);
    a = shot.Update(0, true, false, true, 70);
    assert(a.flywheel);
    a = shot.Update(0, true, false, false, 570);
    assert(!a.flywheel);
    a = shot.Update(1, true, false, false, 580);
    assert(a.flywheel && !a.feed);
    a = shot.Update(1, true, true, false, 590);
    assert(a.feed);
    a = shot.Update(1, false, true, false, 600);
    assert(!a.flywheel && !a.feed);
    a = shot.Update(2, true, true, false, 610);
    assert(!a.single); // 失联后必须先见到 0 才重新允许单发

    shot.Reset();
    shot.Update(0, true, false, false, 1000);
    a = shot.Update(2, true, true, false, 1010);
    assert(a.single);
    a = shot.Update(2, true, true, true, 1020);
    assert(!a.abortSingle);
    a = shot.Update(2, true, true, true, 3011);
    assert(a.abortSingle && !a.flywheel && !a.feed && !a.single);
    a = shot.Update(2, true, true, false, 3020);
    assert(!a.flywheel && !a.single);
    a = shot.Update(0, true, true, false, 3030);
    assert(!a.flywheel);
    a = shot.Update(2, true, true, false, 3040);
    assert(a.single);

    float actualYaw = 1000.0f;
    float actualPitch = 0.0f;
    float yawGoal = actualYaw;
    float pitchGoal = actualPitch;
    for (int i = 0; i < 400; ++i)
    {
        const float imuYaw = (actualYaw - 1000.0f) * 360.0f / 8192.0f;
        const float imuPitch = actualPitch * 180.0f / 3.1415926f;
        assert(VisionAim(10.0f, 20.0f, imuYaw, imuPitch, actualYaw, actualPitch, yawGoal, pitchGoal));
        actualYaw += (yawGoal - actualYaw - 2.0f * 8192.0f / 360.0f) * 0.5f;
        actualPitch += (pitchGoal - actualPitch - 2.0f * 3.1415926f / 180.0f) * 0.5f;
    }
    assert(std::fabs((actualYaw - 1000.0f) * 360.0f / 8192.0f - 10.0f) <= 1.0f);
    assert(std::fabs(actualPitch * 180.0f / 3.1415926f - 20.0f) < 0.3f);

    yaw = 5.0f;
    pitch = 0.0f;
    assert(VisionAim(0.0f, 0.0f, 0.0f, 0.0f, 8190.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - 5.0f) < 0.1f);
    assert(VisionAim(2.0f, 0.0f, 0.0f, 0.0f, 8190.0f, 0.0f, yaw, pitch));
    assert(std::fabs(yaw - (8197.0f + 0.1f * 8192.0f / 360.0f)) < 0.1f);

    float yawMotor = 1000.0f;
    float yawSetpoint = yawMotor;
    float pitchSetpoint = 0.0f;
    float history[1001]{};
    history[0] = yawMotor;
    float low = 100.0f;
    float high = -100.0f;
    for (int i = 0; i < 1000; ++i)
    {
        const float delayedYaw = (history[i < 12 ? 0 : i - 12] - 1000.0f) * 360.0f / 8192.0f;
        assert(VisionAim(10.0f, 0.0f, delayedYaw, 0.0f, yawMotor, 0.0f, yawSetpoint, pitchSetpoint));
        yawMotor += (yawSetpoint - yawMotor) * 0.08f;
        history[i + 1] = yawMotor;
        if (i > 800)
        {
            const float actualYaw = (yawMotor - 1000.0f) * 360.0f / 8192.0f;
            low = std::fmin(low, actualYaw);
            high = std::fmax(high, actualYaw);
        }
    }
    assert(high - low < 0.5f);
    assert(std::fabs((yawMotor - 1000.0f) * 360.0f / 8192.0f - 10.0f) < 0.5f);

    float pitchMotor = 0.0f;
    pitchSetpoint = 0.0f;
    float pitchHistory[1001]{};
    low = 100.0f;
    high = -100.0f;
    yaw = 1000.0f;
    for (int i = 0; i < 1000; ++i)
    {
        const float delayedPitch = pitchHistory[i < 12 ? 0 : i - 12];
        assert(VisionAim(0.0f, 10.0f, 0.0f, delayedPitch, 1000.0f, pitchMotor, yaw, pitchSetpoint));
        pitchMotor += (pitchSetpoint - pitchMotor) * 0.08f;
        pitchHistory[i + 1] = pitchMotor * 180.0f / 3.1415926f;
        if (i > 800)
        {
            low = std::fmin(low, pitchHistory[i + 1]);
            high = std::fmax(high, pitchHistory[i + 1]);
        }
    }
    assert(high - low < 0.5f);
    assert(std::fabs(pitchHistory[1000] - 10.0f) < 0.3f);
}
