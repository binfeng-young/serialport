//
// Created by binfeng.yang on 2019/5/2.
//

#ifndef BVNAVIGATION_LINE_H
#define BVNAVIGATION_LINE_H

template<typename T>
inline T pow2(T value)
{
    return value*value;
}

template<typename Type>
class Vec2;

/// \brief 2D line
///
/// These line templates are defined for: int (Line2i), float (Line2f), double (Line2d)
template<typename Type>
class Line2x
{
public:
    /// \brief First point on the line
    Vec2<Type> p;

    // \brief Another point on the line
    Vec2<Type> q;

    Line2x() : p(), q() { }
    Line2x(const Line2x<Type> &copy) : p(copy.p), q(copy.q) {}
    Line2x(const Vec2<Type> &point_p, const Vec2<Type> &point_q) : p(point_p), q(point_q) {}
    Line2x(const Vec2<Type> &point_p, Type gradient) : p(point_p), q(static_cast<Type> (1), gradient) {}

    /// \brief Return the intersection of this and other line
    ///
    /// \param second = The second line to use
    /// \param intersect = On Return: true if the lines intersect, false if the lines are parallel
    /// \return The point
    Vec2<Type> get_intersection(const Line2x<Type> &second, bool &intersect) const;

    Line2x<Type> &operator = (const Line2x<Type>& copy) { p = copy.p; q = copy.q; return *this; }

    bool operator == (const Line2x<Type>& line) const { return ((p == line.p) && (q == line.q)); }

    bool operator != (const Line2x<Type>& line) const { return ((p != line.p) || (q != line.q)); }
};

typedef Line2x<int> Line2i;
typedef Line2x<float> Line2f;
typedef Line2x<double> Line2d;

// \brief 2D line segment
///
/// A line segment has a start point and an end point\n
/// These line templates are defined for: int (LineSegment2i), float (LineSegment2f), double (LineSegment2d)
template<typename Type>
class LineSegment2x
{
public:
    /// \brief Start point on the line
    Vec2<Type> p;

    // \brief End point on the line
    Vec2<Type> q;

    LineSegment2x() : p(), q() {}
    LineSegment2x(const LineSegment2x<Type> &copy) : p(copy.p), q(copy.q) {}
    LineSegment2x(const Vec2<Type> &point_p, const Vec2<Type> &point_q) : p(point_p), q(point_q) {}

    /// \brief Get the midpoint of this line
    ///
    /// \return The midpoint
    Vec2<Type> get_midpoint() const { return Vec2<Type>((q.x + p.x) / ((Type)2), (q.y + p.y) / ((Type)2)); };

    /// \brief Return the distance from a point to a line.
    ///
    /// \param point = The point.
    Type point_distance(const Vec2<Type> &point);

    /// \brief Return true if two line segments are collinear. (All points are on the same line.)
    ///
    /// \param second = The second line to check with
    /// \return true = They are collinear
    bool collinear(const LineSegment2x<Type> &second) const;

    /// \brief Return true if two line segments intersect.
    ///
    /// \param second = Second line.
    /// \param collinear_intersect = true if a collision is reported when all points are on the same line.
    /// \return true = Intersects
    bool intersects(const LineSegment2x<Type> &second, bool collinear_intersect) const;

    /// \brief Return the intersection point of two lines.
    ///
    /// \param second = Second line.
    /// \param intersect = On Return: The intercept. If the lines are parallel, this contains this line's first point
    /// \return true if the lines intersect, false if the lines are parallel
    Vec2<Type> get_intersection(const LineSegment2x<Type> &second, bool &intersect) const;

    /// \brief Return [<0, 0, >0] if the Point P is right, on or left of the line trough A,B
    ///
    /// \param point = The point
    /// \return Value representing - left (>0), centre (=0), or right (<0)
    Type point_right_of_line(const Vec2<Type> &point) const { return (q.x - p.x) * (point.y - p.y) - (point.x - p.x) * (q.y - p.y); }

    /// \brief Return the normal vector of the line from point A to point B.
    ///
    /// When using Point2i, the vector is an 8 bit fraction (multiplied by 256)
    ///
    /// \return The normal vector
    Vec2<Type> normal() const;

    /// \brief = operator.
    LineSegment2x<Type> &operator = (const LineSegment2x<Type>& copy) { p = copy.p; q = copy.q; return *this; }

    /// \brief == operator.
    bool operator == (const LineSegment2x<Type>& line) const { return ((p == line.p) && (q == line.q)); }

    /// \brief != operator.
    bool operator != (const LineSegment2x<Type>& line) const { return ((p != line.p) || (q != line.q)); }
};
typedef LineSegment2x<int> LineSegment2i;
typedef LineSegment2x<float> LineSegment2f;
typedef LineSegment2x<double> LineSegment2d;






#endif //BVNAVIGATION_LINE_H
