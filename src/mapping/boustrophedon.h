//
// Created by binfeng.yang on 2019/5/27.
//

#ifndef SERIALPORT_BOUSTROPHEDON_H
#define SERIALPORT_BOUSTROPHEDON_H

#include <vector>
#include <map>
#include "cell_index.h"
#include "line.h"
namespace bv {
namespace mapping {
class Costmap2D;
}

class Boustrophedon {
public:
    Boustrophedon(std::shared_ptr<mapping::Costmap2D> costmap);
public:
    void decomposition();

    void fillCell(int number_of_line, int cell_number);

    void update();

    void getNextLine();

    void processLine(int line, std::vector<Vec2i> &obstacles_list, std::vector<Vec2i> &free_list);

    std::vector<int> discontinuousSets(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &linst_two);

    bool relativeContinuity(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &list_two);

    bool isConnection(const Vec2i& one, const Vec2i& two);

    std::vector<Vec2i> getConnection(const std::vector<Vec2i> &list_one, const std::vector<Vec2i> &list_two);

    int findConnection(const std::vector<Vec2i>& connection, int i, bool one);

    std::map<int, std::vector<bv::mapping::CellIndex>> getPath() {
        return path_sets_;
    };


private:

    std::shared_ptr<mapping::Costmap2D> map_;

    int line_number_;
    int line_cur_;
    int line_next_;
    int cell_number_;
    std::vector<Vec2i> obstacles_cur_list_;
    std::vector<Vec2i> obstacles_next_list;
    std::vector<Vec2i> free_cur_list_;
    std::vector<Vec2i> free_next_list_;
    std::vector<int> previous_cell_list_;
    std::map<int /*id*/, std::vector<bv::mapping::CellIndex> /*path*/> path_sets_;


};
}
#endif //SERIALPORT_BOUSTROPHEDON_H
