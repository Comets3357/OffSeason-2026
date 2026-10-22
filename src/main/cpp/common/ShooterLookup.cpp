#include "common/ShooterLookup.h"

#include <cmath>
#include <iterator>

void ShooterLookup::Init() {
    // Distance in meters -> hood angle in degrees.
    angleMap = {{1.85, 4}, {4.49, 6.8}};
    // Distance in meters -> flywheel RPM.
    velocityMap = {{1.85, -2000.0}, {4.49, -2500.0}};
}

double ShooterLookup::calculateHoodAngle(units::meter_t distance) const {
    return interpolate(angleMap, distance.value());
}

double ShooterLookup::calculateFlywheelVelocity(units::meter_t distance) const {
    return interpolate(velocityMap, distance.value());
}

double ShooterLookup::interpolate(const std::map<double, double>& table, double distance) {
    if (table.empty() || !std::isfinite(distance)) return 0.0;
    if (distance <= table.begin()->first) return table.begin()->second;
    if (distance >= table.rbegin()->first) return table.rbegin()->second;

    const auto upper = table.upper_bound(distance);
    const auto lower = std::prev(upper);
    const double fraction = (distance - lower->first) / (upper->first - lower->first);
    return lower->second + fraction * (upper->second - lower->second);
}
