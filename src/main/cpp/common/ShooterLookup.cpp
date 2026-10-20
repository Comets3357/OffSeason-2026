#include "common/ShooterLookup.h"

#include <cmath>
#include <iterator>

void ShooterLookup::Init() {
    // Distance in meters -> hood angle in degrees.
    angleMap = {{1.46, 17.5}, {1.92, 21.0}, {3.29, 28.0}, {5.9, 29.5}};
    // Distance in meters -> flywheel RPM.
    velocityMap = {{1.46, 990.0}, {2.0, 1025.0}, {3.0, 1200.0}, {3.7, 1310.0},
                   {4.0, 1350.0}, {5.0, 1510.0}, {7.0, 2000.0}, {8.0, 2200.0}};
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
