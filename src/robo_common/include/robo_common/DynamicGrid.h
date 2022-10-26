#ifndef DYNAMICGRID_H_
#define DYNAMICGRID_H_

#include <unordered_map>

#include "robo_common/layout/field/FieldPos.h"

template<typename T>
class DynamicGrid
{
public:
    bool contains(FieldPos const & pos) const
    {
        return m_data.find(pos) != m_data.end();
    }
    T const & at(FieldPos const & pos) const
    {
        return m_data.at(pos);
    }
    // TODO: implement proper operator []
    T & at(FieldPos const & pos)
    {
        return m_data[pos];
    }
    auto cbegin() const
    {
        return m_data.cbegin();
    } 
    auto cend() const
    {
        return m_data.cend();
    } 
private:
    std::unordered_map<FieldPos, T> m_data;
};



#endif // DYNAMICGRID_H_
