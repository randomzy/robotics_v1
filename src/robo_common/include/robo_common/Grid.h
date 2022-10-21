#ifndef FIELDMAP_H_
#define FIELDMAP_H_

#include <cinttypes>
#include <vector>
#include <cassert>
#include <sstream>

#include "robo_common/layout/field/FieldPos.h"

template<typename T>
class Grid
{
public:
    Grid(int32_t rows, int32_t cols, T const & value = T{})
        : m_rows(rows)
        , m_cols(cols)
    {
        m_data.resize(m_rows*m_cols, value);
    }
    T const & at(FieldPos const & pos) const
    {
        return at(pos.row, pos.col);
    }
    T & at(FieldPos const & pos)
    {
        return const_cast<T&>(const_cast<Grid const *>(this)->at(pos));
    }
    T const & at(int32_t row, int32_t col) const
    {
        assert(row >= 0 && row < m_rows && col >= 0 && col < m_cols);
        return m_data[col + row*m_cols];
    }
    T  & at(int32_t row, int32_t col)
    {
        return const_cast<T&>(const_cast<Grid const *>(this)->at(row, col));
    }
    int32_t rows() const
    {
        return m_rows;
    }
    int32_t cols() const
    {
        return m_cols;
    }
    std::vector<T> const & asVector() const
    {
        return m_data;
    }
private:
    std::vector<T> m_data;
    int32_t m_rows{};
    int32_t m_cols{};
};

template<typename T>
std::string toString(Grid<T> const & grid)
{
    std::stringstream ss;
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            ss << grid.at(r,c);
        }
        ss << "\n";
    }
    return ss.str();
}

#endif // FIELDMAP_H_
