//
// Created by binfeng.yang on 2019/5/14.
//

#ifndef SERIALPORT_MAP_THREAD_H
#define SERIALPORT_MAP_THREAD_H
#include <QThread>
#include <QtCore/QTime>
#include <mapping/cell_index.h>
namespace bv {
namespace mapping {
class Costmap2D;
}
}
class MapThread : public QThread {
Q_OBJECT
public:
    MapThread(int size_x, int size_y);
    ~MapThread(){}

    void bfs(const bv::mapping::CellIndex& cur_index);
    void dfs(const bv::mapping::CellIndex& cur_index, int dir);
signals:
    void updateCurPose(int x, int y, int theta);
    void drawMovePath(int x1, int y1, int x2, int y2);
    void drawPoseData(int x, int y, int theta, int type);
protected:
    void run() override ;
private:
    std::shared_ptr<bv::mapping::Costmap2D> map_;
    qreal scale_;
    bv::mapping::CellIndex start_index_ = bv::mapping::CellIndex(0, 0);
    std::vector<int> vis_;
    bv::mapping::CellIndex last_pose_index_;
};

#endif //SERIALPORT_MAP_THREAD_H
