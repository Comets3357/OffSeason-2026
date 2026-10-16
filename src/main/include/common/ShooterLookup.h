#pragma once
#include <iostream>
// #include <unordered_map>
#include <map>
#include <units/length.h>
#include <units/angle.h>

using namespace std;
class ShooterLookup
{
    public:
        void Init();
        /**
         * @param distance Distance from target in meters
         */
        double calculateHoodAngle(units::meter_t distance);
        /**
         * @param distance Distance from target in meters
         */
        double calculateFlywheelVelocity(units::meter_t distance);
        /**
         * @param distance Distance from target in meters
         */
        double calculateFlightTime(units::meter_t distance);
        /**
         * @param distance Distance from target in meters
         */
        double calculateVelocityComp(units::meter_t distance);

        /**
         * @param distance Distance from target in meters
         */
        double calculateVelocityScalarComp(units::meter_t distance);
        /**
         * @param angle Orientation of the turret
         */
        double calculateVelocityScalarComp(units::degree_t angle);

        /**
         * @param distance Distance from target in meters
         */
        double calculateAngleComp(units::meter_t distance);
        /**
         * @param angle Orientation of the turret
         */
        double calculateAngleComp(units::degree_t angle);

    private:

        // Generic function to perform lookup and interpolation for all maps
        double performLookup(std::map<double, double> &map, units::meter_t &distance);

        // Generic function to perform lookup and interpolation for all maps
        double performUnitCircleLookup(std::map<units::degree_t, double> &map, units::degree_t &angle);
        
        // Helper function for linear interpolation
        double interpolate(double &lowerKey, double &upperKey, double &lowerValue, double &upperValue, units::meter_t distance);

    map<double, double> angleMap;
    map<double, double> velocityMap;
    map<double, double> velocityCompMap;
    map<double, double> velocityScalarCompMap;
    map<double, double> angleCompMap;
    map<double, double> timeMap;

    map<units::degree_t, double> velocityScalarCompensation;
    map<units::degree_t, double> angleScalarCompensation;
};