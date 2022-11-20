#ifndef MATH_H_
#define MATH_H_

#include <tuple>
#include <tf2/LinearMath/Vector3.h>

struct Line
{
    tf2::Vector3 a;
    tf2::Vector3 b;
};

inline auto decomposeAngleAxis(tf2::Vector3 const & angleAxis)
{
    auto axis = angleAxis.normalized();
    auto angle = angleAxis.length();
    return std::make_tuple(angle, axis);
}

inline auto distance(Line const & l, tf2::Vector3 p)
{
    assert(!(l.a - l.b).isZero());

    auto e = l.b - l.a;
    auto dist = tf2::tf2Cross((p - l.a), (p - l.b)).length()/e.length();
    auto t = -tf2::tf2Dot(l.a - p, e)/e.length2();
    return std::make_tuple(dist, l.a + (l.b - l.a)*t);
}

inline auto distance(Line const & l1, Line const & l2)
{
    assert(!(l1.a - l1.b).isZero());
    assert(!(l2.a - l2.b).isZero());

    auto e1 = l1.b - l1.a;
    auto e2 = l2.b - l2.a;
    auto n = tf2::tf2Cross(e1, e2);
    if (n.isZero()) {
        // lines are parallel, so take distance from any point
        // on l2 to l1
        return distance(l1, l2.a);
    }
    auto dist = std::abs(tf2::tf2Dot(n, (l1.a - l2.b)))/n.length();
    auto t1 = tf2::tf2Dot(tf2::tf2Cross(e2, n), l2.a - l1.a)/tf2::tf2Dot(n, n);
    // Comments are left for completeness
    // auto t2 = tf2::tf2Dot(tf2::tf2Cross(e1, n), l2.a - l1.a)/tf2::tf2Dot(n, n);
    auto p1 = l1.a + e1*t1;
    // auto p2 = l2.a + e2*t2;
    return std::make_tuple(dist, p1);
}

#endif // MATH_H_
