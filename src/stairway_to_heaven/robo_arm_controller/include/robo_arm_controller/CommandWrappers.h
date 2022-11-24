#ifndef COMMANDWRAPPERS_H_
#define COMMANDWRAPPERS_H_

#include <array>
#include <iostream>
#include <variant>
#include <vector>
#include <fstream>
#include <sstream>

#include <tf2/LinearMath/Vector3.h>

namespace cw{
    constexpr size_t joint_count = 6;
    constexpr auto indent = "  ";
}

using JointDescription = std::array<float, cw::joint_count>;

struct MovejCommand
{
    JointDescription pose{};
    float acceleration{0.f};
    float velocity{0.f};
};

struct MovelCommand
{
    tf2::Vector3 point{};
    tf2::Vector3 angleAxis{};
    float acceleration {0.f};
    float velocity {0.f};
    float radius {0.f};
};

struct InitGrippersCommand{};
struct OpenGrippersCommand{};
struct CloseGrippersCommand{};

inline std::ostream & operator << (std::ostream & os, MovejCommand const & mjc)
{
    os << cw::indent << "movej([";
    const char * delim = "";
    for (const auto & j : mjc.pose) {
        os << delim << j; 
        delim = ",";
    }
    os << "],a=" << mjc.acceleration;
    os << ",v=" << mjc.velocity;
    os << ")\n";
    return os;
}

inline std::ostream & operator << (std::ostream & os, MovelCommand const & mlc)
{
    os << cw::indent << "movel(";
    os << "p[" << mlc.point.x() << "," << mlc.point.y() << "," << mlc.point.z();
    os << "," << mlc.angleAxis.x() << "," << mlc.angleAxis.y() << "," << mlc.angleAxis.z() << "]";
    os << "," << "a=" << mlc.acceleration << ",v=" << mlc.velocity << ",t=0,r=" << mlc.radius;
    os << ")\n";
    return os;
}

inline std::ostream & operator << (std::ostream & os, [[maybe_unused]]InitGrippersCommand const & igc)
{
    os << cw::indent << "rq_activate_and_wait()\n";
    return os;
}

inline std::ostream & operator << (std::ostream & os, [[maybe_unused]]OpenGrippersCommand const & igc)
{
    os << cw::indent << "rq_open_and_wait()\n";
    return os;
}

inline std::ostream & operator << (std::ostream & os, [[maybe_unused]]CloseGrippersCommand const & igc)
{
    os << cw::indent << "rq_close_and_wait()\n";
    return os;
}

using MoveCommand = std::variant<MovejCommand, MovelCommand, InitGrippersCommand, OpenGrippersCommand, CloseGrippersCommand>;

inline std::string robotDo(std::vector<MoveCommand> const & commands, std::string const & header)
{
    std::stringstream ss;
    ss << header;
    for (const auto & command : commands) {
        std::visit([&ss](auto && cmd){
            ss << cmd;
        }, command);
    }
    ss << "end" << std::endl;
    return ss.str();
}

inline std::string to_string(tf2::Vector3 const & v)
{
    std::stringstream ss;
    ss << v.x() << "," << v.y() << "," << v.z();
    return ss.str();
}

inline std::string toCSV(MovelCommand const & ml)
{
    return to_string(ml.point*100);
}

inline void writeCSV(std::vector<MoveCommand> const & commands, std::string const & csvFilename)
{
    std::ofstream ofs(csvFilename);
    for (const auto & command : commands) {
        std::visit([&ofs](auto const & cmd){
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, MovelCommand>) {
                ofs << toCSV(cmd) << "\n";
            }
        }, command);
    }
}

#endif // COMMANDWRAPPERS_H_
