#include "common/ShooterLookup.h"
#include <cmath>

void ShooterLookup::Init() {

    // key: distance (meters)
    // value: angle (degrees)
    angleMap[1.46] = 17.5;
    angleMap[1.92] = 21;
    angleMap[3.29] = 28;
    angleMap[5.9] = 29.5;
    
    // key: distance (meters)
    // value: velocity (rpm)
    velocityMap[1.46] = 990.0;
    velocityMap[2.0] = 1025.0;
    velocityMap[3.0] = 1200.0;
    velocityMap[3.7] = 1310.0;
    velocityMap[4.0] = 1350.0;
    velocityMap[5.0] = 1510.0; 
    velocityMap[7] = 2000.0;
    velocityMap[8] = 2200.0;

    //depending on the angle, scale the velocity map by a given ratio
    velocityScalarCompensation[0_deg] = 1.00;
    velocityScalarCompensation[45_deg] = 1.0254;
    velocityScalarCompensation[90_deg] = 1.0455;
    velocityScalarCompensation[135_deg] = 1.0254;
    velocityScalarCompensation[180_deg] = 1.00;
    velocityScalarCompensation[225_deg] = 0.929;
    velocityScalarCompensation[270_deg] = 0.909;
    velocityScalarCompensation[315_deg] = 0.926;

    //key: distance (meters)
    //value:: difference in angle (degrees)
    angleCompMap[2.0] = 10.0;
    angleCompMap[3.0] = 12.0;
    angleCompMap[4.0] = 13;
    angleCompMap[5.0] = 14; 
    angleCompMap[5.9] = 15;

    //depending on the angle, scale the angle comp map by a given ratio
    angleScalarCompensation[0_deg] = -0.5;
    angleScalarCompensation[45_deg] = -0.28;
    angleScalarCompensation[90_deg] = 0.25;
    angleScalarCompensation[135_deg] = 0.78;
    angleScalarCompensation[180_deg] = 1.0;
    angleScalarCompensation[225_deg] = 0.78;
    angleScalarCompensation[270_deg] = 0.25;
    angleScalarCompensation[315_deg] = -0.28;

    // key: distance (meters)
    // value: time of flight (seconds)
    timeMap[1.3] = 1.02;
    timeMap[2.6] = 1.22;
    timeMap[3.3] = 1.29;
    timeMap[5.6] = 1.39;
}

double ShooterLookup::calculateHoodAngle(units::meter_t distance) {
    return performLookup(angleMap, distance);
}

double ShooterLookup::calculateFlywheelVelocity(units::meter_t distance) {
    return performLookup(velocityMap, distance);
}

double ShooterLookup::calculateFlightTime(units::meter_t distance) {
    return performLookup(timeMap, distance);
}

double ShooterLookup::calculateVelocityComp(units::meter_t distance) {
    return performLookup(velocityCompMap, distance);
}

double ShooterLookup::calculateVelocityScalarComp(units::meter_t distance) {
    if (velocityScalarCompMap.empty()) return 1.0;
    return performLookup(velocityScalarCompMap, distance);
}

double ShooterLookup::calculateVelocityScalarComp(units::degree_t angle) {
    return performUnitCircleLookup(velocityScalarCompensation, angle);
}

double ShooterLookup::calculateAngleComp(units::meter_t distance) {
    return performLookup(angleCompMap, distance);
}

double ShooterLookup::calculateAngleComp(units::degree_t angle) {
    return performUnitCircleLookup(angleScalarCompensation, angle);
}

double ShooterLookup::performLookup(std::map<double, double> &map, units::meter_t &distance) {
    //Clamp to the table endpoints and handle unused compensation tables.
    if (map.empty()) return 0.0;
    if (distance.value() <= map.begin()->first) return map.begin()->second;
    if (distance.value() >= map.rbegin()->first) return map.rbegin()->second;

    auto upperIt = map.upper_bound(distance.value());
    auto lowerIt = std::prev(upperIt);

    if (upperIt != map.end() && lowerIt != map.end())
    {
        double lowerDistance = lowerIt->first;
        double upperDistance = upperIt->first;
        double lowerValue = lowerIt->second;
        double upperValue = upperIt->second;
        return interpolate(lowerDistance, upperDistance, lowerValue, upperValue, distance);
    }

    //return a negative value is something goes wrong
    return -1;
}

double ShooterLookup::interpolate(double &lowerKey, double &upperKey, double &lowerValue, double &upperValue, units::meter_t distance) 
{
    return ((upperValue - lowerValue) / (upperKey - lowerKey)) * (distance.value() - lowerKey) + lowerValue;
}

double ShooterLookup::performUnitCircleLookup(std::map<units::degree_t, double> &map, units::degree_t &angle) {

    if (map.empty()) return 0.0;
    if (map.size() == 1) return map.begin()->second;

    //Wrap angles and interpolate across the 315-to-360 degree seam too.
    double wrapped = std::fmod(angle.value(), 360.0);
    if (wrapped < 0) wrapped += 360.0;
    auto upper = map.upper_bound(units::degree_t{wrapped});
    auto lower = upper == map.begin() ? std::prev(map.end()) : std::prev(upper);
    if (upper == map.end()) upper = map.begin();

    double lowerAngle = lower->first.value();
    double upperAngle = upper->first.value();
    if (upperAngle <= lowerAngle) upperAngle += 360.0;
    if (wrapped < lowerAngle) wrapped += 360.0;
    double fraction = (wrapped - lowerAngle) / (upperAngle - lowerAngle);
    return lower->second + fraction * (upper->second - lower->second);
}

