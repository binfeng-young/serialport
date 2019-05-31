//
// Created by binfeng.yang on 2019/5/3.
//

#ifndef BVNAVIGATION_POINTX_H
#define BVNAVIGATION_POINTX_H
#include <limits>
template<typename Type>
struct Pointx {
    union { Type x; Type s; };
    union { Type y; Type t; };


    Pointx(Type x_in, Type y_in) : x(x_in), y(y_in)
    {}

    Pointx()
    {
        x = 0;
        y = 0;
    }
    static bool is_equal(const Pointx<Type> &first, const Pointx<Type> &second, Type epsilon)
    {
        Type diff_x = second.x - first.x; Type diff_y = second.y - first.y;
        return (diff_x >= -epsilon && diff_x <= epsilon && diff_y >= -epsilon && diff_y <= epsilon);
    }

    /// \brief Returns true if equal within the bounds of an epsilon
    ///
    /// \param other = Other value
    /// \param epsilon = The epsilon (eg FLT_EPSILON, DBL_EPSILON)
    bool is_equal(const Pointx<Type> &other, Type epsilon = std::numeric_limits<Type>::epsilon()) const { return Pointx<Type>::is_equal(*this, other, epsilon); }

    /// \brief Returns the distance between two points
    ///
    /// \param other = Other value
    /// \return Distance between two points
    Type dist(const Pointx<Type>& other) const { return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y); }

    bool operator == (const Pointx<Type>& other) const { return is_equal(other); }

    bool operator != (const Pointx<Type>& other) const { return !this->operator==(other); }

    bool operator < (const Pointx<Type>& other) const { return y < other.y || (y == other.y && x < other.x); }

    Pointx<Type> operator + (const Pointx<Type> &other) const { return {x + other.x, y + other.y}; }

    Pointx<Type> operator - (const Pointx<Type> &other) const { return {x - other.x, y - other.y}; }
};

typedef Pointx<int> Point_2i;
typedef Pointx<float> Point_2f;
typedef Pointx<double> Point_2d;
typedef Point_2i Vec2i;
#endif //BVNAVIGATION_POINTX_H
