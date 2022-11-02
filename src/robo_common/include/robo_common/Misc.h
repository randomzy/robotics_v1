#ifndef MISC_H_
#define MISC_H_

#include "robo_common/defines/RoboCommonDefines.h"

#include <robo_common/layout/field/FieldUtils.h>
#include <utils/Log.h>
#include <queue>

inline Direction rotateCW(Direction const & direction)
{
    switch (direction) {
        case Direction::UP: return Direction::RIGHT;
        case Direction::RIGHT: return Direction::DOWN;
        case Direction::DOWN: return Direction::LEFT;
        case Direction::LEFT: return Direction::UP;
        default:
            LOGERR("Invalid direction");
            return Direction::UP;
    }
}

inline Direction rotateCC(Direction const & direction)
{
    switch (direction) {
        case Direction::UP: return Direction::LEFT;
        case Direction::LEFT: return Direction::DOWN;
        case Direction::DOWN: return Direction::RIGHT;
        case Direction::RIGHT: return Direction::UP;
        default:
            LOGERR("Invalid direction");
            return Direction::UP;
    }
}

inline std::array<FieldPos, RoboCommonDefines::SURROUNDING_TILES_CTN> getSurroundingPos(RobotState const & robotState)
{
    return std::array<FieldPos, RoboCommonDefines::SURROUNDING_TILES_CTN> {{
        FieldUtils::getAdjacentPos(rotateCC(robotState.dir), robotState.fieldPos),
        FieldUtils::getAdjacentPos(robotState.dir, robotState.fieldPos),
        FieldUtils::getAdjacentPos(rotateCW(robotState.dir), robotState.fieldPos),
    }};
};

inline bool canStepOn(uint8_t marker)
{
    return
        marker != RoboCommonDefines::Markers::BIG_OBSTACLE_MARKER &&
        marker != RoboCommonDefines::Markers::SMALL_OBSTACLE_MARKER &&
        marker != RoboCommonDefines::Markers::FIELD_OUT_OF_BOUND_MARKER;
}

inline FieldPos updatePosition(MoveType move, FieldPos position, Direction direction)
{
    if (move==MoveType::FORWARD)
        return FieldUtils::getAdjacentPos(direction, position);
    return
        position;
}

template<typename QF>
inline void moveToNeighbour(RobotState const & state, FieldPos const & dest, QF queryfunction)
{
    auto const & src = state.fieldPos;
    auto const & dir = state.dir;

    assert( (std::abs(src.col - dest.col) <= 1 && src.row == src.row) ||
            (std::abs(src.row - dest.row) <= 1 && src.col == src.col));
    
    // TOOD: there must be an intelligent way to do this, maybe with rotations
    // local coordinate system to global 
    bool dest_down = src.row < dest.row;
    bool dest_up = src.row > dest.row;
    bool dest_left = src.col > dest.col;
    bool dest_right = src.col < dest.col;

    switch(dir)
    {
        case Direction::UP:
            if(dest_up) {
                // dir = Direction::UP;
            }
            if(dest_right) {
                // dir = Direction::RIGHT;
                queryfunction(MoveType::ROTATE_RIGHT);
            }
            if(dest_down) {
                // dir = Direction::DOWN;
                queryfunction(MoveType::ROTATE_LEFT);
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_left) {
                // dir = Direction::LEFT;
                queryfunction(MoveType::ROTATE_LEFT);
            }
            break;
        case Direction::RIGHT:
            if(dest_up) {
                // dir = Direction::UP;
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_right) {
                // dir = Direction::RIGHT;
            }
            if(dest_down) {
                // dir = Direction::DOWN;
                queryfunction(MoveType::ROTATE_RIGHT);
            }
            if(dest_left) {
                // dir = Direction::LEFT;
                queryfunction(MoveType::ROTATE_LEFT);
                queryfunction(MoveType::ROTATE_LEFT);
            }
            break;
        case Direction::DOWN:
            if(dest_up) {
                // dir = Direction::UP;
                queryfunction(MoveType::ROTATE_LEFT);
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_right) {
                // dir = Direction::RIGHT;
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_down) {
                // dir = Direction::DOWN;
            }
            if(dest_left) {
                // dir = Direction::LEFT;
                queryfunction(MoveType::ROTATE_RIGHT);
            }
            break;
        case Direction::LEFT:
            if(dest_up) {
                // dir = Direction::UP;
                queryfunction(MoveType::ROTATE_RIGHT);
            }
            if(dest_right) {
                // dir = Direction::RIGHT;
                queryfunction(MoveType::ROTATE_LEFT);
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_down) {
                // dir = Direction::DOWN;
                queryfunction(MoveType::ROTATE_LEFT);
            }
            if(dest_left) {
                // dir = Direction::LEFT;
            }
            break;
        default:
            LOGERR("Invalid Direction");
            break;
    }
    if (dest_up || dest_down || dest_left || dest_right) {
        queryfunction(MoveType::FORWARD);
    }
};

template<bool TrimBorder = true>
FieldMap convertDynamicFieldMap(DynamicFieldMap const & dynamicGrid)
{
    auto minmax_col = std::minmax_element(dynamicGrid.cbegin(), dynamicGrid.cend(), [](const auto & a, const auto & b){
        return a.first.col < b.first.col;
    });
    auto minmax_row = std::minmax_element(dynamicGrid.cbegin(), dynamicGrid.cend(), [](const auto & a, const auto & b){
        return a.first.row < b.first.row;
    });

    int32_t min_row = (*minmax_row.first).first.row;
    int32_t max_row = (*minmax_row.second).first.row;

    int32_t min_col = (*minmax_col.first).first.col;
    int32_t max_col = (*minmax_col.second).first.col;

    int32_t rows = max_row - min_row + 1;
    int32_t cols = max_col - min_col + 1;

    if constexpr (TrimBorder) {
        assert(rows >= 2 && cols >= 2);
        // subtract -2 beacuse first and last row/col are borders
        rows -= 2;
        cols -= 2;
        min_row += 1;
        min_col += 1;
        max_row -= 1;
        max_col -= 1;
    }    
    FieldMap fieldMap(rows, cols);

    for (int r = min_row; r <= max_row; r++) {
        for (int c = min_col; c <= max_col; c++) {
            if (dynamicGrid.contains({r,c})) {
                fieldMap.at(r - min_row, c - min_col) = dynamicGrid.at({r,c});
            } else {
                fieldMap.at(r - min_row, c - min_col) = 'X';
            }
        }
    }
    return fieldMap;
}

#endif // MISC_H_
