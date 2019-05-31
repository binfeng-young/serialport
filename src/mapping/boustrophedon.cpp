//
// Created by binfeng.yang on 2019/5/27.
//

#include <iostream>
#include <algorithm>
#include "boustrophedon.h"
#include "costmap_2d.h"
using namespace bv;
using namespace mapping;

Boustrophedon::Boustrophedon(std::shared_ptr<Costmap2D> costmap)
{
    map_ = std::move(costmap);
    line_number_ = 0;
    cell_number_ = 0;
}


void Boustrophedon::fillCell(int number_of_line, int cell_number)
{

    const Vec2i& line = free_next_list_[number_of_line];

    if (path_sets_.find(cell_number) == path_sets_.end()) {
        path_sets_.insert(std::make_pair(cell_number, std::vector<CellIndex>()));
    }
    auto &path = path_sets_[cell_number];
    if ((path.size() / 2) % 2) {
        path.emplace_back(line.t, line_number_);
        path.emplace_back(line.s, line_number_);
    } else {
        path.emplace_back(line.s, line_number_);
        path.emplace_back(line.t, line_number_);
    }
    for (int i = 0; i < (line.t - line.s + 1); i ++) {
        map_->setCost(CellIndex(line.s + i, line_number_), cell_number);
    }
}
void Boustrophedon::decomposition()
{
    while (line_number_ < map_->getLimits().size_y - 1) {
        update();
        //relContinuity
        if (relativeContinuity(obstacles_cur_list_, obstacles_next_list)) {
            auto connect_free = getConnection(free_cur_list_, free_next_list_);
            std::vector<int> actual;
            for (int i = 0; i < free_next_list_.size(); i++) {
                size_t j = findConnection(connect_free, i, false);
                if (j == connect_free.size()) {
                    cell_number_++;
                    fillCell(i, cell_number_);
                    actual.push_back(cell_number_);
                } else {
                    int pre_i = connect_free[j].s;
                    fillCell(i, previous_cell_list_[pre_i]);
                    actual.push_back(previous_cell_list_[pre_i]);
                }
            }
            previous_cell_list_ = actual;
            continue;
        }
        //discontinuity
        auto dis_obstacles = discontinuousSets(obstacles_cur_list_, obstacles_next_list);
        bool cur_continuity = true;
        int num = 0;
        std::vector<int> actual;
        for (int i = 1; i < obstacles_next_list.size(); i++) {
            bool next_continuity = cur_continuity;
            cur_continuity = (std::find(dis_obstacles.begin(), dis_obstacles.end(), i) == dis_obstacles.end());
            if (!(cur_continuity && next_continuity)) {
                cell_number_++;
                actual.push_back(cell_number_);
                fillCell(i - 1, cell_number_);
                for (int k = num; k < free_cur_list_.size(); k++) {
                    if (free_next_list_[i - 1].t >= free_cur_list_[k].s) {
                        num++;
                    }
                }
                continue;
            }

            if (isConnection(free_cur_list_[num], free_next_list_[i - 1])) {
                int jump = 0;
                for (int k = num; k < free_cur_list_.size(); k++) {
                    if (free_next_list_[i - 1].t >= free_cur_list_[k].s) {
                        jump++;
                    } else break;
                }
                if (jump == 1) {
                    fillCell(i - 1, previous_cell_list_[num]);
                    actual.push_back(previous_cell_list_[num]);
                } else { //out
                    cell_number_++;
                    fillCell(i - 1, cell_number_);
                    actual.push_back(cell_number_);
                }
                num += jump;
            } else if (free_next_list_[i - 1].t < free_cur_list_[num].s) {
                cell_number_++;
                fillCell(i - 1, cell_number_);
                actual.push_back(cell_number_);
            } else {
                bool connect = false;
                int k = num;
                for (; k < free_cur_list_.size(); k++) {
                    if (isConnection(free_cur_list_[k], free_next_list_[i - 1])) {
                        connect = true;
                        break;
                    }
                }
                if (connect) {
                    fillCell(i - 1, previous_cell_list_[k]);
                    actual.push_back(previous_cell_list_[k]);
                } else {
                    cell_number_++;
                    fillCell(i - 1, cell_number_);
                    actual.push_back(cell_number_);
                }
            }
        }
        previous_cell_list_ = actual;
    }
}

void Boustrophedon::update()
{
    line_number_ ++;
    getNextLine();
    processLine(line_cur_, obstacles_cur_list_, free_cur_list_);
    processLine(line_next_, obstacles_next_list, free_next_list_);

}

void Boustrophedon::getNextLine()
{
    auto limits = map_->getLimits();
    if (line_number_ >= limits.size_y) {
        line_cur_ = line_next_;
    } else {
        line_cur_ = line_number_ - 1;
        line_next_ = line_number_;
    }
}


void Boustrophedon::processLine(int line, std::vector<Vec2i> &obstacles_list, std::vector<Vec2i> &free_list)
{

    obstacles_list.clear();
    bool cur_obstacles = false;
    bool obstacles_line_add = false;
    Vec2i obs_line;

    bool cur_free = false;
    bool free_line_add = false;
    Vec2i free_line;
    free_list.clear();


    for (int i = 0; i < map_->getLimits().size_x; i ++) {
        auto pre_obstacles = cur_obstacles;
        auto cost = map_->getCost(CellIndex(i, line));
        cur_obstacles = cost == MapValue::LETHAL_OBSTACLE || cost == MapValue::INSCRIBED_INFLATED_OBSTACLE;
        if (!pre_obstacles && cur_obstacles) {
//            obstacles_num ++;
            obs_line.s = i;
            obstacles_line_add = true;
        } else if (pre_obstacles && !cur_obstacles) {
            obs_line.t = i - 1;
            obstacles_list.push_back(obs_line);
            obstacles_line_add = false;
        }

        auto pre_free = cur_free;
        cur_free = !cur_obstacles;
        if (!pre_free && cur_free) {
//            obstacles_num ++;
            free_line.s = i;
            free_line_add = true;
        } else if (pre_free && !cur_free) {
            free_line.t = i - 1;
            free_list.push_back(free_line);
            free_line_add = false;
        }
    }
    if (obstacles_line_add) {
        obs_line.t = map_->getLimits().size_x - 1;
        obstacles_list.push_back(obs_line);
    }

    if (free_line_add) {
        free_line.t = map_->getLimits().size_x - 1;
        free_list.push_back(free_line);
    }

}

bool Boustrophedon::relativeContinuity(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &list_two)
{
    std::vector<Vec2i> connection = getConnection(list_one, list_two);

    for (int i = 0; i < list_one.size(); i ++) {
        if (findConnection(connection, i, true) >= connection.size()) {
            return false;
        }
    }
    for (int i = 0; i < list_two.size(); i++) {
        if (findConnection(connection, i, false) >= connection.size()) {
            return false;
        }
    }
    return true;
}

std::vector<int> Boustrophedon::discontinuousSets(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &list_two)
{
    std::vector<Vec2i> connection = getConnection(list_one, list_two);
    std::vector<int> discontinuous;
    for (int i = 0; i < list_two.size(); i++) {
        if (findConnection(connection, i, false) >= connection.size()) {
            discontinuous.push_back(i);
        }
    }
    return discontinuous;
}

bool Boustrophedon::isConnection(const Vec2i& one, const Vec2i& two)
{
    return ((one.s <= two.s && two.s <= one.t) || (one.s <= two.t && (two.t <= one.t)))
    || ((two.s <= one.s && one.s <= two.t) || (two.s <= one.t && one.t <= two.t));
}

int Boustrophedon::findConnection(const std::vector<Vec2i>& connection, int i, bool one)
{
    int size = connection.size();
    for (int j = 0; j < size; j ++) {
        int val = one ? connection[j].s : connection[j].t;
        if (val == i) {
            return j;
        }
    }
    return size;
}

std::vector<Vec2i> Boustrophedon::getConnection(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &list_two)
{
    std::vector<Vec2i> connection;
    for (int i = 0; i < list_one.size(); i++) {
        for (int j = 0; j < list_two.size(); j++) {
            if (isConnection(list_one[i], list_two[j])) {
                connection.emplace_back(i, j);
            }
        }
    }
    return connection;
}