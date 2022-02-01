#pragma once

namespace Intake
{
    enum class DIRECTION {
        OUT,
        OFF,
        IN
    };

    void init();
    void drive(DIRECTION mode);
    void deploy(bool val);

    [[nodiscard]] bool isIntakeDown();
} // namespace Intake