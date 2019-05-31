#include <grid_path_ros.h>
#include <cell_index.h>
#include <algorithm>
#include <stdio.h>
#include <iostream>
using std::cout;
using std::endl;

bool GridPath_ros::getPath(const std::vector<float> &potential,
                           int start_x, int start_y,
                           int end_x, int end_y, std::vector<std::pair<float, float>> &path)
{
    std::pair<float, float> current;
    /*current.first = end_x;
    current.second = end_y;*/
    current.first = end_x;
    current.second = end_y;

    //cout << "current_x: " << current.second << " y: " << current.first << endl;
    //int start_index = getIndex(start_x, start_y);
    int start_index = getIndex(start_x, start_y);

    //cout <<  "start index: " << start_index << endl;
    path.push_back(current);
    int c = 0;
    int ns = xs_ * ys_;

    while (getIndex(current.first, current.second) != start_index)
    {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++)
        {
            for (int yd = -1; yd <= 1; yd++)
            {
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd;
                int y = current.second + yd;
                int index = getIndex(x, y);
                //int index = getIndex(y, x);
                if (potential[index] < min_val)
                {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)
            return false;
        /*current.first = min_x;
        current.second = min_y;*/
        current.first = min_x;
        current.second = min_y;
        //cout << "current_x: " << current.second << " y: " << current.first << endl;
        path.push_back(current);

        if (c++ > ns * 4)
        {
            return false;
        }

    }
    return true;
}


