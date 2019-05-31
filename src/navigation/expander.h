//
// Created by peter on 3/29/19.
//
#pragma once

#include "potential_calculator.h"

#define POT_HIGH 1.0e10        // unassigned cell potential

class Expander
{
public:
    Expander(PotentialCalculator *p_calc, int nx, int ny) :
        do_not_pass_unknown_cell_(true), lethal_cost_(254), neutral_cost_(50), factor_(3.0), p_calc_(p_calc)
    {
        setSize(nx, ny);
    }

    virtual bool calculatePotentials(const std::vector<uint8_t> &costs,
                                     int start_x, int start_y, int end_x, int end_y,
                                     int cycles, std::vector<float> &potential) = 0;

    /**
     * @brief  Sets or resets the size of the map
     * @param nx The x size of the map
     * @param ny The y size of the map
     */
    virtual void setSize(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    } /**< sets or resets the size of the map */
    void setLethalCost(unsigned char lethal_cost)
    {
        lethal_cost_ = lethal_cost;
    }

    void setNeutralCost(unsigned char neutral_cost)
    {
        neutral_cost_ = neutral_cost;
    }

    void setFactor(float factor)
    {
        factor_ = factor;
    }

    void setDoNotPassUnknownCell(bool do_not_pass_unknown_cell)
    {
        do_not_pass_unknown_cell_ = do_not_pass_unknown_cell;
    }

    void clearEndpoint(unsigned char *costs, std::vector<float> &potential, int gx, int gy, int s)
    {
        int startCell = toIndex(gx, gy);
        for (int i = -s; i <= s; i++)
        {
            for (int j = -s; j <= s; j++)
            {
                int n = startCell + i + nx_ * j;
                if (potential[n] < POT_HIGH)
                    continue;
                float c = costs[n] + neutral_cost_;
                float pot = p_calc_->calculatePotential(potential, c, n);
                potential[n] = pot;
            }
        }
    }

protected:
    inline int toIndex(int x, int y)
    {
        return x + nx_ * y;
    }

    int nx_, ny_, ns_; /**< size of grid, in pixels */
    bool do_not_pass_unknown_cell_;
    unsigned char lethal_cost_, neutral_cost_;
    int cells_visited_;
    float factor_;
    PotentialCalculator *p_calc_;

};
