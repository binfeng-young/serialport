//
// Created by peter on 3/29/19.
//

#include <iostream>

#include "astar.h"
#include "cell_index.h"
#include "bv_type.h"
#include "navigation.h"

using std::cout;
using std::endl;
using namespace bv::mapping;

AStarExpansion::AStarExpansion(PotentialCalculator *p_calc, int xs, int ys) :
    Expander(p_calc, xs, ys)
{
}

bool
AStarExpansion::calculatePotentials(const std::vector<uint8_t> &costs,
                                    int start_x, int start_y, int end_x, int end_y,
                                    int cycles, std::vector<float> &potential)
{
    queue_.clear();
    //cout << "nx: " << nx_ << " ny: " << ny_ << endl;
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));
    //std::fill(potential, potential + ns_, POT_HIGH);
    potential = std::vector<float>(ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);

    //cout << "start_i: " << start_i  << " cost: "  << (int)costs[start_i] << endl;
    //cout << "goal_i: " << goal_i  << " cost: "  << (int)costs[goal_i] << endl;

    //test
    //show global map on opencv
    CellIndex start{start_x, start_y};
    CellIndex goal{end_x, end_y};
    std::vector<std::pair<float, float>> path;
    //
    // show_global_map(costs, nx_, ny_, start, goal, path);

    int cycle = 0;
    while (queue_.size() > 0 && cycle < cycles)
    {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(const std::vector<uint8_t> &costs,
                         std::vector<float> &potential, float prev_potential,
                         int next_i, int end_x, int end_y)
{
    //std::cout << "costs[next_i]: " << int(costs[next_i]) << endl;
    if (next_i < 0 || next_i >= ns_)
    {
        //cout << "out of bound: " << ns_ << endl;
        return;
    }

    if (potential[next_i] < POT_HIGH)
    {
        //cout << "potential[next_i] has been set: " << next_i << endl;
        return;
    }

    if (costs[next_i] == lethal_cost_)
    {
        //std::cout << "obstacle: " << int(costs[next_i]) << endl;
        return;
    }


    if(costs[next_i] == MapValue::UNKNOWN && do_not_pass_unknown_cell_)
    {
        //cout << "unknown: " << int(costs[next_i]) << endl;
        return;
    }

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_,
                                                    next_i, prev_potential);
    //cout << "potential[next_i]: " << potential[next_i] << endl;

    int x = next_i % nx_;
    int y = next_i / nx_;
    //cout << "next_x: " << x << " y: " << y << endl;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}