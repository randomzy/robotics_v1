#ifndef ROBO_COMMON_FIELDPOS_H_
#define ROBO_COMMON_FIELDPOS_H_

//System headers
#include <cstdint>
#include <functional>

//Other libraries headers

//Own components headers

//Forward declarations

struct FieldPos {
  FieldPos() = default;
  FieldPos(int32_t row, int32_t col);

  bool operator==(const FieldPos& other) const;
  bool operator<(const FieldPos& other) const;
  FieldPos operator + (FieldPos const & other);

  int32_t row { 0 };
  int32_t col { 0 };
};

std::string toString(FieldPos const & fieldPos);

template<>
struct std::hash<FieldPos>
{
    size_t operator()(FieldPos const & c) const noexcept
    {
        uint64_t hash = c.col;
        hash <<= sizeof(int32_t) * 4;
        hash ^= c.row;
        return std::hash<uint64_t>()(hash);
    }
};

#endif /* ROBO_COMMON_FIELDPOS_H_ */
