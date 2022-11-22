#ifndef CONFIG_H_
#define CONFIG_H_

#include "ros2_game_engine/communicator/Ros2Communicator.h"
#include "tf2/LinearMath/Vector3.h"

struct RoboArmConfig
{
    Ros2CommunicatorConfig ros2CommunicatorConfig;

    // order in config determines the order boxes will be picked
    std::vector<tf2::Vector3> boxes = {
        {0.491, -0.134, 0.050}, // Box10
        {0.491, -0.134, -0.060}, // Box1
        {0.690, -0.143, -0.060}, // Box4
        {0.888, -0.144, 0.050}, // Box13
        {0.888, -0.144, -0.060}, // Box7
        {0.701, 0.039, 0.050}, // Box12
        {0.487, 0.042, -0.060}, // Box2
        {0.701, 0.039, -0.060}, // Box5
        {0.900, 0.044, -0.060}, // Box8
        {0.491, 0.240, 0.050}, // Box11
        // {0.887, 0.226, 0.050}, // Box14
        {0.491, 0.240, -0.060}, // Box3
        {0.491, 0.240, -0.060}, // Box3
        {0.690, 0.230, -0.060}, // Box6
        {0.887, 0.226, -0.060}, // Box9
    };

    double boxDim{0.11}; // in meters
    // change in box size due to deformity
    double boxDelta{0.003}; // in meters

    tf2::Vector3 boxesOrientation{2.224, -2.224, 0.0};
    tf2::Vector3 placeOrientation{0, 3.148, 0.0};

    // tf2::Vector3 towerPlaceOrientation{0.732, 1.756, -1.757};
    tf2::Vector3 towerPlaceOrientation{0.001, -2.222, 2.222};
    // tf2::Vector3 towerPlaceOrientation{2.354, 2.434, -2.434};

    double staircaseRotation{0.0}; // in degrees
    tf2::Vector3 positionOfFirstStep{0.07, -0.70, -0.060}; // in meters

    // additional waypoints when approaching staircase target
    tf2::Vector3 approachPlace{0.15 ,0.0, 0.20}; // in meters
    tf2::Vector3 approachPlace2{0.0 ,0.0, 0.20}; // in meters
    // additional waypoint when approaching tower
    tf2::Vector3 approachPlace3{0.30 ,0.05, 0.00}; // in meters
    tf2::Vector3 approachPick{0.0 ,0.0, 0.20}; // in meters

    float acceleration{1.1}; // in m/s^2
    float velocity{1.3}; // in m/s
    float radius{0.1}; // in meters

    // if line segment is in the cylinder defined by Oz and safeOzRadius
    // bisect the segment at the point closest to Oz, and move the point outward
    // so that its projection on XY plane has lenght of safeOzRadius
    double safeOzRadius{0.30}; // in meters

    bool isSimulation{false};

    tf2::Vector3 homePoint{-0.171, -0.682, 0.428};
    tf2::Vector3 homeAngleAxis{0.0, 3.148, 0};

    std::string csvFilename = "points.csv"; // dumps waypoints
};

#endif // CONFIG_H_
