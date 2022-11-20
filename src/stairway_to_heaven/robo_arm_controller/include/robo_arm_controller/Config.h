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
        {0.887, 0.226, 0.050}, // Box14
        {0.491, 0.240, -0.060}, // Box3
        {0.690, 0.230, -0.060}, // Box6
        {0.887, 0.226, -0.060}, // Box9
    };

    double boxDim{0.11}; // in meters

    double staircaseRotation{0.0}; // in degrees
    tf2::Vector3 positionOfFirstStep{-0.350, -0.75,-0.060}; // in meters

    tf2::Vector3 approachOffset{0.0 ,0.0, 0.11}; // in meters
    tf2::Vector3 placeDelta{0,0,0.005}; // in meters

    float acceleration{1.0};
    float velocity{1.0};

    double safeOzRadius{0.35}; // in meters

    bool isSimulation{true};

    tf2::Vector3 homePoint{-0.171, -0.682, 0.428};
    tf2::Vector3 homeAngleAxis{0.0, 3.148, 0};

    std::string csvFilename = "points.csv"; // dumps waypoints
};

#endif // CONFIG_H_
