//
// Created by binfeng.yang on 2019/5/14.
//

#include <queue>
#include <iostream>
#include <stack>
#include <random>
#include "map_thread.h"
#include "mapping/costmap_2d.h"
#include <random>
using namespace bv::mapping;
MapThread::MapThread(int size_x, int size_y)
{
    auto limits = CellLimits(size_x, size_y);
    map_ = std::make_shared<Costmap2D>(limits, 0.05, 0, false);
    vis_ = std::vector<int>(size_x * size_y, 0);
}


void MapThread::bfs(const CellIndex& cur_index)
{
    struct Id {
        CellIndex index;
        int dis{};
        bool operator<(const Id &x) const
        {
            return dis > x.dis;
        }

    };
    last_pose_index_ = cur_index;
    int max_val = 0;
    //cost_map_->deleteMaps();
    std::priority_queue<Id> queue;
    queue.push(Id{cur_index, 1});
    map_->setCost(cur_index, 1);
    while (!queue.empty()) {
        auto now = queue.top().index;
        auto dis = queue.top().dis;
        queue.pop();
        auto nears = nhood8(now, map_->getLimits());
        for (auto near : nears) {
            auto cost = map_->getCost(near);
            if (cost) {
                continue;
            }
            if (near.x != now.x && near.y == now.y) {
                cost = dis;
            } else {
                cost = dis + 1;
            }
            //QGraphicsRectItem *itemRect = gridMap_[mapSize_.width() -1 - near.y][mapSize_.height() -1 - near.x];
            //QColor color(255 - cost, 255 - cost, 255 - cost);
            //itemRect->setBrush(QBrush(color));
            //itemRect->setPen(QPen(color));
            map_->setCost(near, cost);
            if (max_val < cost) {
                start_index_ = near;
            }
            queue.push(Id{near, cost});
        }
    }

    auto bound = [=](int val, int min, int max) {
        if (val < min) {
            return min;
        }
        if (val > max) {
            return max;
        }
        return val;
    };
    auto limit = map_->getLimits();
    std::random_device rd;
    for (int i = 0; i < 20; i++) {
        int x = rd() % limit.size_x;
        int y = rd() % limit.size_y;
        for (int j = -2; j < 2; j++) {
            for (int p = -2; p < 2; p++) {
                int xx = bound(x + j, 0, limit.size_x-1);
                int yy = bound(y + p, 0, limit.size_y-1);
                map_->setCost(CellIndex(xx, yy), 0);
                emit drawPoseData(xx, yy, 0, 2);
            }
        }
    }
}
static short direction[8][2] = {1, 0, -1, 0, 0, 1, 0, -1, 1, 1, 1, -1, -1, 1, -1, -1};
void MapThread::dfs(const CellIndex& cur_index, int dir) {
    auto id = map_->getLimits().toId(cur_index);
    if (!map_->contains(cur_index) || vis_.at(id)) {
        return;
    }
        //std::cout << cur_index.x << " " << cur_index.y << std::endl;
    vis_[id] = 1;
//        emit updateCurPose(cur_index.x, cur_index.y, 0);
    //emit drawMovePath(last_pose_index_.x, last_pose_index_.y, cur_index.x, cur_index.y);
    last_pose_index_ = cur_index;
    //QThread::msleep(10);
    //auto nears = nhood4(cur_index, map_->getLimits());
    for (const auto &i : direction) {
        CellIndex next = CellIndex(cur_index.x + i[0], cur_index.y + i[1]);
        if (isContains(next, map_->getLimits())) {
           dfs(next, dir);
        }
    }
    return;
};

void MapThread::run()
{
    bfs(CellIndex(map_->getLimits().size_x / 2, map_->getLimits().size_y / 2));
    struct Id {
        CellIndex index;
        int dir_id{};
    };
    auto check_vis = [&](const CellIndex& index) {
        auto cur_id = map_->getLimits().toId(index);
        return vis_.at(cur_id) || map_->getCost(index) == 0;
    };
    std::stack<Id> st;
    auto dir_id = -1;
    st.push(Id{start_index_, dir_id});
    CellIndex cur_index = start_index_;
    //int val = map_->getCost(cur_index);

    while (!st.empty()) {
        if (dir_id >= 7 || !map_->contains(cur_index)) {
            cur_index = st.top().index;
            dir_id = st.top().dir_id;
            st.pop();
            continue;
        }
        if (dir_id == -1) {
            if (check_vis(cur_index)) {
                cur_index = st.top().index;
                dir_id = st.top().dir_id;
                st.pop();
                continue;
            }
            auto cur_id = map_->getLimits().toId(cur_index);
            vis_[cur_id] = 1;

            emit updateCurPose(cur_index.x, cur_index.y, 0);
            emit drawPoseData(cur_index.x, cur_index.y, 0, 0);
            emit drawMovePath(last_pose_index_.x, last_pose_index_.y, cur_index.x, cur_index.y);
            last_pose_index_ = cur_index;
        }

        st.push(Id{cur_index, dir_id + 1});

        cur_index = cur_index + CellIndex(direction[dir_id + 1][0], direction[dir_id + 1][1]);
        if (!map_->contains(cur_index) || check_vis(cur_index)) {
            cur_index = st.top().index;
            dir_id = st.top().dir_id;
            st.pop();
            continue;
        }
        dir_id = -1;
        QThread::msleep(10);
    }


   /* std::function<void(const CellIndex& cur_index, int dir)> _dfs;

*/
    dfs(start_index_, 0);
}

