#pragma once
#include <vector>
#include <algorithm>
#include "cell_index.h"
#include "expander.h"

class Index
{
public:
    Index(int a, float b)
    {
        i = a;
        cost = b;
    }

    int i;
    float cost;
};

struct greater1
{
    bool operator()(const Index &a, const Index &b) const
    {
        return a.cost > b.cost;
    }
};

class AStarExpansion : public Expander
{
public:
    AStarExpansion(PotentialCalculator *p_calc, int nx, int ny);

    bool
    calculatePotentials(const std::vector<uint8_t> &costs,
                        int start_x, int start_y, int end_x, int end_y, int cycles,
                        std::vector<float> &potential);

private:
    void add(const std::vector<uint8_t> &costs,
             std::vector<float> &potential, float prev_potential, int next_i, int end_x, int end_y);

    std::vector<Index> queue_;
};

