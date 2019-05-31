/*
 * Created by bfyoung on 2019/2/14.
 */

#ifndef _INC_TYPEDEF_H_
#define _INC_TYPEDEF_H_

namespace bv {
namespace mapping {

struct Point {
    double x;
    double y;
};

struct Pose2D {
    Point point;
    double theta;
};

struct Point2i {
    int x;
    int y;
};

struct Line_t {
    float a;
    float b;
    float c;
};
typedef struct Line_t Line;

}
}

//float angle_between_lines(Line la, Line lb);

#endif /* _TYPEDEF_H_ */
