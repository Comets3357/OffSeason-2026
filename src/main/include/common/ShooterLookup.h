#pragma once

#include <map>
#include <units/length.h>

class ShooterLookup {
public:
    void Init();
    double calculateHoodPosition(units::meter_t distance) const; // Internal motor turns.
    double calculateFlywheelVelocity(units::meter_t distance) const; // RPM.

private:
    static double interpolate(const std::map<double, double>& table, double distance);
    std::map<double, double> hoodPositionMap;
    std::map<double, double> velocityMap;
};
