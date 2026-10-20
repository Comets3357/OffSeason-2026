#pragma once

#include <map>
#include <units/length.h>

class ShooterLookup {
public:
    void Init();
    double calculateHoodAngle(units::meter_t distance) const; // Degrees.
    double calculateFlywheelVelocity(units::meter_t distance) const; // RPM.

private:
    static double interpolate(const std::map<double, double>& table, double distance);
    std::map<double, double> angleMap;
    std::map<double, double> velocityMap;
};
