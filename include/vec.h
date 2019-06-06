//
// Created by binfeng.yang on 2019/5/3.
//

#ifndef BVNAVIGATION_VEC_H
#define BVNAVIGATION_VEC_H
#include <limits>
template<typename Type>
class Vec2;

template<typename Type>
class Vec3;

template<typename Type>
struct Vec2 {
    union { Type x; Type s; Type r;};
    union { Type y; Type t; Type g;};

    Vec2() : x(0), y(0) { }
    Vec2(Type x_in, Type y_in) : x(x_in), y(y_in) {}
   /* Vec2(Type scalar) : x(scalar), y(scalar) { }
    //Vec2(Vec2<Type> copy) { x = copy.x; y = copy.y; }
    Vec2(Vec3<Type> copy) { x = copy.x; y = copy.y; }*/


    static bool is_equal(const Vec2<Type> &first, const Vec2<Type> &second, Type epsilon)
    {
        Type diff_x = second.x - first.x; Type diff_y = second.y - first.y;
        return (diff_x >= -epsilon && diff_x <= epsilon && diff_y >= -epsilon && diff_y <= epsilon);
    }

    /// \brief Returns true if equal within the bounds of an epsilon
    ///
    /// \param other = Other value
    /// \param epsilon = The epsilon (eg FLT_EPSILON, DBL_EPSILON)
    bool is_equal(const Vec2<Type> &other, Type epsilon = std::numeric_limits<Type>::epsilon()) const { return Vec2<Type>::is_equal(*this, other, epsilon); }

    /// \brief Returns the distance between two points
    ///
    /// \param other = Other value
    /// \return Distance between two points
    Type dist(const Vec2<Type>& other) const { return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y); }

    bool operator == (const Vec2<Type>& other) const { return is_equal(other); }

    bool operator != (const Vec2<Type>& other) const { return !this->operator==(other); }

    bool operator < (const Vec2<Type>& other) const { return y < other.y || (y == other.y && x < other.x); }

    Vec2<Type> operator + (const Vec2<Type> &other) const { return {x + other.x, y + other.y}; }

    Vec2<Type> operator - (const Vec2<Type> &other) const { return {x - other.x, y - other.y}; }
};

typedef Vec2<int> Vec2i;
typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;
typedef Vec2i Point_2i;
typedef Vec2f Point_2f;
typedef Vec2d Point_2d;

template<typename Type>
struct Vec3 {
    union { Type x; Type s; Type r;};
    union { Type y; Type t; Type g;};
    union { Type z; Type u; Type b;};

    Vec3() : x(0), y(0), z(0) { }
    Vec3(Type x_in, Type y_in, Type z_in) : x(x_in), y(y_in), z(z_in) {}
    //explicit Vec3(Type scalar) : x(scalar), y(scalar), z(scalar) { }
    Vec3(Vec2<Type> copy, const Type z_in) { x = copy.x; y = copy.y; z = z_in; }
    //explicit Vec3(Vec3<Type> copy) { x = copy.x; y = copy.y; z = copy.z; }

    static bool is_equal(const Vec3<Type> &first, const Vec3<Type> &second, Type epsilon)
    {
        Type diff_x = second.x - first.x; Type diff_y = second.y - first.y; Type diff_z = second.z - first.z;
        return (diff_x >= -epsilon && diff_x <= epsilon && diff_y >= -epsilon && diff_y <= epsilon && diff_z >= -epsilon && diff_z >= epsilon);
    }

    /// \brief Returns true if equal within the bounds of an epsilon
    ///
    /// \param other = Other value
    /// \param epsilon = The epsilon (eg FLT_EPSILON, DBL_EPSILON)
    bool is_equal(const Vec3<Type> &other, Type epsilon = std::numeric_limits<Type>::epsilon()) const { return Vec3<Type>::is_equal(*this, other, epsilon); }

    /// \brief Returns the distance between two points
    ///
    /// \param other = Other value
    /// \return Distance between two points
    Type dist(const Vec3<Type>& other) const { return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y) + (other.z - z); }

    bool operator == (const Vec3<Type>& other) const { return is_equal(other); }

    bool operator != (const Vec3<Type>& other) const { return !this->operator==(other); }

    bool operator < (const Vec3<Type>& other) const { return y < other.y || (y == other.y && x < other.x); }

    Vec3<Type> operator + (const Vec3<Type> &other) const { return {x + other.x, y + other.y, z + other.z}; }

    Vec3<Type> operator - (const Vec3<Type> &other) const { return {x - other.x, y - other.y, z - other.z}; }
};
typedef Vec3<int> Vec3i;
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;
typedef Vec3i Point_3i;
typedef Vec3f Point_3f;
typedef Vec3d Point_3d;
#endif //BVNAVIGATION_VEC_H
