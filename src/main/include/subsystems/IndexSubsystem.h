#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include "wrappers/NovaBase.h"

class IndexSubsystem : public frc2::SubsystemBase {
    public:
        IndexSubsystem();

        void Periodic() override;
        void SimulationPeriodic() override;

        void setHopperSpeed(double percent);
        void setKickerSpeed(double percent);
        void setHighSpeedRailSpeed(double percent);

        double getHopperSpeed();
        double getHighSpeedRailSpeed();


    private:
        static constexpr int kKickerCanId = 22;
        NovaMotor HopperMotor{21};
        NovaMotor KickerMotor{kKickerCanId};
        NovaMotor KickerFollower{23};

        double HopperSpeed = 0.0;
        double HighSpeedRailSpeed = 0.0;
};
