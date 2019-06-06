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
#include "footprint.h"
#include "boustrophedon.h"
using namespace bv::mapping;
MapThread::MapThread(int size_x, int size_y)
{
    float robot_radius = 0.16f;
    auto limits = CellLimits(size_x, size_y);
    map_ = std::make_shared<Costmap2D>(limits, 0.07, MapValue::FREE_SPACE, false);
    vis_ = std::vector<int>(size_x * size_y, 0);
    vis2_ = std::vector<int>(size_x * size_y, 0);
    rectangle_ = makeRectangleFootprintFromRadius(robot_radius);
    circle_ = makeFootprintFromRadius(robot_radius);
    semicircle_ = makeObstacleFootprintFromRadius(robot_radius);
    triangle_ = makeTriangleFootprintFromRadius(robot_radius);
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
/*
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
            */
/*if (max_val < cost) {
                start_index_ = near;
            }*//*

            queue.push(Id{near, cost});
        }
    }
*/

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
    auto random = [&]() {
        using namespace std::chrono;
        steady_clock::time_point t1 = steady_clock::now();
        long long int count = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - steady_clock::time_point(nanoseconds ::zero())).count() % 111239;
        return (int)(rd() * (count) % 11335597);
    };
    for (int i = 0; i < 30; i++) {
        int x = random() % limit.size_x;
        int y = random() % limit.size_y;
        Point pt = map_->getWorld(CellIndex(x, y));
        float theta = (random() % 100)  * M_PI / 100;
        std::vector<Point> polygon;
        std::vector<Point>* p[4] = {&circle_, &rectangle_, &semicircle_, &triangle_};
        auto pol = p[random() % 4];
        transformFootprint(pt, theta, *pol, polygon);


        std::vector<CellIndex> map_polygon;

        for (const auto& i : polygon) {
            auto index = map_->getCellIndex(i);
            if (!limit.isContains(index)) {
                continue;
            }
            map_polygon.push_back(index);
        }

        if (map_polygon.empty()) {
            continue;
        }
        std::vector<CellIndex> polygon_cells;

        map_->convexFillCells(map_polygon, polygon_cells);
        for (auto index : polygon_cells) {
            if (!limit.isContains(index)) {
                continue;
            }
            map_->setCost(index, MapValue::LETHAL_OBSTACLE);
            //emit drawPoseData(index.x, index.y, 0, 1);
            ins_.push_back(index);
        }
//        for (int j = 1; j <= 5; j++) {
//            int l = j > 3 ? 6 - j : j;
//            for (int p = -l; p <= l; p++) {
//                int xx = bound(x + j, 0, limit.size_x-1);
//                int yy = bound(y + p, 0, limit.size_y-1);
//                map_->setCost(CellIndex(xx, yy), MapValue::LETHAL_OBSTACLE);
//                emit drawPoseData(xx, yy, 0, 1);
//            }
//        }
    }
//    auto check_vis = [&](const CellIndex& index) {
//        auto cur_id = map_->getLimits().toId(index);
//        return vis_.at(cur_id) || map_->getCost(index) == 0;
//    };
//
//    emit updateCurPose(cur_index.x, cur_index.y, 0);
//    std::queue<CellIndex> free_queue;
//    free_queue.push(cur_index);
//    while (!free_queue.empty()) {
//        auto now_index = free_queue.front();
//        free_queue.pop();
//        auto id = map_->getLimits().toId(now_index);
//        if (vis_[id]) {
//            continue;
//        }
//        vis_[id] = 1;
//        auto nears = map_->getLimits().nhood4(now_index, true);
//        bool frontier = false;
//        for (auto near : nears) {
//            if (!map_->getLimits().isContains(near) || map_->getCost(near) == MapValue::LETHAL_OBSTACLE) {
//                if (frontier) {
//                    continue;
//                }
//                map_->setCost(now_index, MapValue::INSCRIBED_INFLATED_OBSTACLE);
//                emit drawPoseData(now_index.x, now_index.y, 0, 2);
//                frontier = true;
//            } else {
//                free_queue.push(near);
//            }
//        }
//    }

    map_->inflationForObstacle(0.15);

    for (auto cell : CellIndexRangeIterator(limit)) {
        if (map_->getCost(cell) == MapValue::LETHAL_OBSTACLE) {
            emit drawPoseData(cell.x, cell.y, 0, 1);
        }
    }

}
static short direction[8][2] = {{1, 0}, {-1, 0}, {1, 1}, {1, -1}, {0, 1}, {0, -1},  {-1, 1}, {-1, -1}};
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
    CellIndex cur_index = CellIndex(map_->getLimits().size_x / 2, map_->getLimits().size_y / 2);
    bfs(cur_index);
    QThread::sleep(1);
    bv::Boustrophedon bcd(map_);
    bcd.decomposition();
    for (auto cell : CellIndexRangeIterator(map_->getLimits())) {
//        std::cout << map_->getCost(cell) << "\t";
//        if (cell.x >= map_->getLimits().size_x - 1) {
//            std::cout << std::endl;
//        }
        emit drawMap(cell.x, cell.y, map_->getCost(cell));
    }

    auto lines_set = bcd.getLines();

    for (auto& lines_s : lines_set)
    {
        std::vector<CellIndex> path;
        auto &lines = lines_s.second;
        for (int i = 0; i < lines.size(); i ++) {
            auto vec3 = lines[i];
            if ((i / 2) % 2) {
                path.emplace_back(vec3.t, vec3.u, vec3,);
                path.emplace_back(line.s, line_number_);
            } else {
                path.emplace_back(line.s, line_number_);
                path.emplace_back(line.t, line_number_);
            }
        }
    }




    auto path_sets = bcd.getPath();
    for (const auto& path_pair : path_sets) {
        const auto& path = path_pair.second;
        CellIndex cur = path[0];
        for (int k = 1; k < path.size(); k ++) {
            emit drawMovePath(cur.x, cur.y, path[k].x, path[k].y);
            cur = path[k];
        }
    }

    while (1) {
        QThread::sleep(100);
    }
    auto cent_ax = cur_index.y;
    struct Id {
        CellIndex index;
        int dir_id{};
    };
    auto check_vis = [&](const CellIndex& index) {
        auto cur_id = map_->getLimits().toId(index);
        return vis_.at(cur_id) || map_->getCost(index) == 0;
    };
    vis_.clear();
   // while (1);
    std::stack<CellIndex> frontier;
    int head_dir = 1;
    int fold_dir = 1; // left_turn
    auto limit = map_->getLimits();
    CellIndex side_index = CellIndex(-1, -1);
    while (1) {
        emit updateCurPose(cur_index.x, cur_index.y, 0);
        emit drawPoseData(cur_index.x, cur_index.y, 0, 0);
        emit drawMovePath(last_pose_index_.x, last_pose_index_.y, cur_index.x, cur_index.y);
        last_pose_index_ = cur_index;
        map_->setCost(cur_index, MapValue::FREE_SPACE);
        auto next = CellIndex(cur_index.x + head_dir, cur_index.y);
        //int fold_dir =  cur_index.y - cent_ax >= 0 ? -1 : 1;
        bool is_s = false;
        for (int i = -1; i <= 1; i ++) {
            auto next_sid = CellIndex(cur_index.x + i * head_dir, cur_index.y + fold_dir);
            if (map_->getCost(next_sid) == MapValue::NO_INFORMATION) {
                emit drawPoseData(next_sid.x, next_sid.y, 0, 3);
                side_index = next_sid;
                is_s = true;
                frontier.push(next_sid);
            }
        }
       //if (!is_s) {
           /*   fold_dir*=-1;*/
             for (int i = -1; i <= 1; i ++) {
                 auto next_sid = CellIndex(cur_index.x + i * head_dir, cur_index.y + fold_dir * -1);
                 if (map_->getCost(next_sid) == MapValue::NO_INFORMATION) {
                     emit drawPoseData(next_sid.x, next_sid.y, 0, 3);
                     side_index = next_sid;
                     frontier.push(next_sid);
                 }
             }
        //}
        QThread::msleep(20);
        if (map_->getCost(next) == MapValue::LETHAL_OBSTACLE || map_->getCost(next) == INSCRIBED_INFLATED_OBSTACLE/* || map_->getCost(next) == MapValue::FREE_SPACE*/) { // turn
            if (side_index == CellIndex(-1, -1)) {
                while(!frontier.empty()) {
                    CellIndex top = frontier.top();
                    frontier.pop();
                    if (map_->getCost(top) != MapValue::NO_INFORMATION)
                    {
                        continue;
                    }
                    side_index = top;
                    break;
                }
                fold_dir*=-1;
               // while (1);
            }
            cur_index = side_index;
            side_index = CellIndex(-1, -1);
            head_dir *=-1;
            continue;
        }
        if (!limit.isContains(next)) {
            while (1);
        }
        cur_index = next;
    }
    //CellIndex cur_index = CellIndex(map_->getLimits().size_x / 2, map_->getLimits().size_y / 2);
    /*std::stack<CellIndex> st;
    int dir = 0;

    while (cur_index != CellIndex(-1, -1)) {
        int val =  map_->getCost(cur_index);
        emit updateCurPose(cur_index.x, cur_index.y, 0);
        emit drawPoseData(cur_index.x, cur_index.y, 0, 0);
        emit drawMovePath(last_pose_index_.x, last_pose_index_.y, cur_index.x, cur_index.y);
        QThread::msleep(10);
        last_pose_index_ = cur_index;
        vis_[map_->getLimits().toId(cur_index)] = 1;
        auto nears = nhood8(cur_index, map_->getLimits());
        CellIndex next_index = CellIndex(-1, -1);
        int max_cost = std::numeric_limits<int>::max();
        bool eq = false;
        for (auto near : nears) {
            if (check_vis(near))
            {
                continue;
            }
            st.push(near);
            //emit drawPoseData(cur_index.x, cur_index.y, 0, 3);
            if (!eq) {
                auto cost = map_->getCost(near);
                if (val == cost) {
                    next_index = near;

                    eq = true;
                } else if (cost < max_cost) {
                    max_cost = cost;
                    next_index = near;
                }
            }
        }
        if (next_index == CellIndex(-1, -1)) {
            while(!st.empty()) {
                CellIndex top = st.top();
                st.pop();
                if (check_vis(top))
                {
                    continue;
                }
                next_index = top;
                break;
            }
        }
        cur_index = next_index;
    }
*/



    //int val = map_->getCost(cur_index);
/*
 *
    std::stack<Id> st;
    auto dir_id = -1;
    st.push(Id{cur_index, dir_id});
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
    }*/


   /* std::function<void(const CellIndex& cur_index, int dir)> _dfs;

*/
    dfs(start_index_, 0);
}

