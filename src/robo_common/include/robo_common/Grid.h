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
    Grid() = default;
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
        return at(pos.row, pos.col);
    }
    T const & at(int32_t row, int32_t col) const
    {
        assert(row >= 0 && row < m_rows && col >= 0 && col < m_cols);
        return m_data[col + row*m_cols];
    }
    T & at(int32_t row, int32_t col)
    {
        assert(row >= 0 && row < m_rows && col >= 0 && col < m_cols);
        return m_data[col + row*m_cols];
    }
    bool contains(FieldPos const & pos) const
    {
        return pos.col < m_cols && pos.row < m_rows && pos.col >= 0 && pos.row >= 0;
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
    ss << "rows: " << grid.rows() << " cols: " << grid.cols() << std::endl;
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            ss << grid.at(r,c);
        }
        ss << "\n";
    }
    return ss.str();
}

#endif // FIELDMAP_H_
