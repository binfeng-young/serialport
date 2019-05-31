//
// Created by binfeng.yang on 2019/5/2.
//

#include "line.h"
#include "bv_type.h"
#include "cmath"

template<typename Type>
Pointx<Type> Line2x<Type>::get_intersection(const Line2x<Type> &second, bool &intersect) const
{
    Type denominator = (q.x - p.x) * (second.q.y - second.p.y) - (second.q.x - second.p.x) * (q.y - p.y);
    if (denominator == ((Type)0))
    {
        intersect = false;
        return Pointx<Type>();
    }
    intersect = true;

    Type numerator = (second.p.x - p.x) * (q.y - p.y) - (q.x - p.x) * (second.p.y - p.y);

    Pointx<Type> result;
    result.x = (second.p.x) + (numerator * (second.q.x - second.p.x)) / denominator;
    result.y = (second.p.y) + (numerator * (second.q.y - second.p.y)) / denominator;
    return result;
}

template class Line2x<int>;
template class Line2x<float>;
template class Line2x<double>;


template<typename Type>
Pointx<Type> LineSegment2x<Type>::normal() const
{
    Pointx<Type> N;
    N.x = -(q.y - p.y);
    N.y = q.x - p.x;

    Type len = sqrt(N.x*N.x + N.y*N.y);
    N.x /= len;
    N.y /= len;

    return N;
}

// For integers
template<>
Point_2i LineSegment2x<int>::normal() const
{
    LineSegment2f line(Point_2f((float)p.x, (float)p.y), Point_2f((float)q.x, (float)q.y));
    Point_2f n = line.normal();
    Point_2i vec((int)(n.x * 256.0f), (int)(n.y * 256.0f));
    return vec;
}

/*	    From comp.graphics.algorithms FAQ
    +---------------------------------------+

    Let A,B,C,D be 2-space position vectors.  Then the directed line
    segments AB & CD are given by:

    AB=A+r(B-A), r in [0,1]
    CD=C+s(D-C), s in [0,1]

    If AB & CD intersect, then

    A+r(B-A)=C+s(D-C), or

    Ax+r(Bx-Ax)=Cx+s(Dx-Cx)
    Ay+r(By-Ay)=Cy+s(Dy-Cy)  for some r,s in [0,1]

    Solving the above for r and s yields

    (Ay-Cy)(Dx-Cx)-(Ax-Cx)(Dy-Cy)
    r = -----------------------------  (eqn 1)
    (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)

    (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
    s = -----------------------------  (eqn 2)
    (Bx-Ax)(Dy-Cy)-(By-Ay)(Dx-Cx)

    Let P be the position vector of the intersection point, then

    P=A+r(B-A) or

    Px=Ax+r(Bx-Ax)
    Py=Ay+r(By-Ay)

    By examining the values of r & s, you can also determine some
    other limiting conditions:

    If 0<=r<=1 & 0<=s<=1, intersection exists
    r<0 or r>1 or s<0 or s>1 line segments do not intersect

    If the denominator in eqn 1 is zero, AB & CD are parallel
    If the numerator in eqn 1 is also zero, AB & CD are collinear.
    */

template<typename Type>
Pointx<Type> LineSegment2x<Type>::get_intersection(const LineSegment2x<Type> &second, bool &intersect) const
{
    Type denominator = ((q.x - p.x)*(second.q.y - second.p.y) - (q.y - p.y)*(second.q.x - second.p.x));

    if (denominator == ((Type)0))
    {
        intersect = false;
        return p;
    }

    Type r = ((p.y - second.p.y)*(second.q.x - second.p.x) - (p.x - second.p.x)*(second.q.y - second.p.y)) / denominator;
    Type s = ((p.y - second.p.y)*(q.x - p.x) - (p.x - second.p.x)*(q.y - p.y)) / denominator;

    // We use the open interval [0;1) or (0;1] depending on the direction of CD
    if (second.p.y < second.q.y)
    {
        if (!((s >= ((Type)0) && s < ((Type)1)) && (r >= ((Type)0) && r <= ((Type)1))))
        {
            intersect = false;
            return p;
        }
    }
    else
    {
        if (!((s > ((Type)0) && s <= ((Type)1)) && (r >= ((Type)0) && r <= ((Type)1))))
        {
            intersect = false;
            return p;
        }
    }

    intersect = true;
    return Pointx<Type>(p.x + r*(q.x - p.x), p.y + r*(q.y - p.y));
}

template<typename Type>
bool LineSegment2x<Type>::intersects(const LineSegment2x<Type> &second, bool collinear_intersect) const
{
    Type denominator = ((q.x - p.x)*(second.q.y - second.p.y) - (q.y - p.y)*(second.q.x - second.p.x));

    if (denominator == ((Type)0)) // parallel
    {
        if ((p.y - second.p.y)*(second.q.x - second.p.x) - (p.x - second.p.x)*(second.q.y - second.p.y) == ((Type)0)) // collinear
        {
            if (collinear_intersect)
                return true;
            else
                return false;
        }
        return false;
    }

    Type r = ((p.y - second.p.y)*(second.q.x - second.p.x) - (p.x - second.p.x)*(second.q.y - second.p.y)) / denominator;
    Type s = ((p.y - second.p.y)*(q.x - p.x) - (p.x - second.p.x)*(q.y - p.y)) / denominator;

    // We use the open interval [0;1) or (0;1] depending on the direction of CD
    if (second.p.y < second.q.y)
    {
        if ((s >= ((Type)0) && s < ((Type)1)) && (r >= ((Type)0) && r <= ((Type)1)))
            return true;
    }
    else
    {
        if ((s > ((Type)0) && s <= ((Type)1)) && (r >= ((Type)0) && r <= ((Type)1)))
            return true;
    }

    return false;
}


// Collinear points are points that all lie on the same line.
template<typename Type>
bool LineSegment2x<Type>::collinear(const LineSegment2x<Type> &second) const
{
    Type denominator = ((q.x - p.x)*(second.q.y - second.p.y) - (q.y - p.y)*(second.q.x - second.p.x));
    Type numerator = ((p.y - second.p.y)*(second.q.x - second.p.x) - (p.x - second.p.x)*(second.q.y - second.p.y));

    if (denominator == ((Type)0) && numerator == ((Type)0))
        return true;

    return false;
}

/* ----- from comp.graphics.algorithms FAQ ------

  dist to line (Ax,Ay->Bx,By) from point x,y

  L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 )

  (Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
  r = -------------------------------
  L^2
  */

template<typename Type>
Type LineSegment2x<Type>::point_distance(const Pointx<Type> &point)
{
    Type L = pow2(q.x - p.x) + pow2(q.y - p.y);
    Type r = ((point.x - p.x)*(q.x - p.x) + (point.y - p.y)*(q.y - p.y)) / L;

    if (r <= 0 || r >= 1)
    {
        return std::min(point.dist(p), point.dist(q));
    }

    Type s = ((p.y - point.y)*(q.x - p.x) - (p.x - point.x)*(q.y - p.y)) / L;

    s *= sqrt(L);

    if (s < ((Type)0))
        s = -s;

    return s;
}

// For integers
template<>
int LineSegment2x<int>::point_distance(const Point_2i &point)
{
    int L = pow2(q.x - p.x) + pow2(q.y - p.y);
    int r = ((point.x - p.x)*(q.x - p.x) + (point.y - p.y)*(q.y - p.y)) / L;

    if (r <= 0 || r >= 1)
    {
        return std::min(point.dist(p), point.dist(q));
    }

    int s = ((p.y - point.y)*(q.x - p.x) - (p.x - point.x)*(q.y - p.y)) / L;

    s *= (int)(sqrt((float)L) + 0.5f);

    if (s < 0)
        s = -s;

    return s;
}

template class LineSegment2x<int>;
template class LineSegment2x<float>;
template class LineSegment2x<double>;
