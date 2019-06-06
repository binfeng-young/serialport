//
// Created by binfeng.yang on 2019/5/27.
//

#ifndef SERIALPORT_BOUSTROPHEDON_H
#define SERIALPORT_BOUSTROPHEDON_H

#include <vector>
#include <map>
#include <queue>
#include "cell_index.h"
#include "line.h"
namespace bv {
namespace mapping {
class Costmap2D;
}
class Graph {
public:
    struct Edge {
        int to;
        int next;
        Edge(int v, int nt) : to(v), next(nt) {}
    };

    //typedef std::vector<int> TableType;

public:

    Graph(int n)
        : head_(std::vector<int>(n, -1))
        , vis_(std::vector<bool>(n, false)) {}

    void addEdge(int from, int to)
    {
        edges_.emplace_back(to, head_[from]);
        head_[from] = edges_.size() - 1;

        edges_.emplace_back(from, head_[to]);
        head_[to] = edges_.size() - 1;
        /* edges_.emplace_back(from, to);
         edges_.emplace_back(to, from);
         int m = edges_.size();
         graph_[from].push_back(m - 2);
         graph_[to].push_back(m - 1);*/
    }

    void dfs(int u, std::vector<int>& path)
    {
        vis_[u] = true;
        path.push_back(u);

        //for (auto e : graph_[u]) {
        for (int e = head_[u]; e != -1; e = edges_[e].next) {
            auto v = edges_[e].to;
            if (vis_[v]) {
                continue;
            }
            dfs(v, path);
        }
    }

    std::vector<int> bfs(int start) {
        std::vector<int> path;
        std::queue<int> q;
        q.push(start);
        while (!q.empty()) {
            auto u = q.front();
            q.pop();
            path.push_back(u);
            if (vis_[u]) {
                continue;
            }
            vis_[u] = true;
            //for (auto e : graph_[u]) {
            for (int e = head_[u]; e != -1; e = edges_[e].next) {
                q.push(edges_[e].to);
            }
        }
        return path;
    }


private:
    std::vector<Edge> edges_;
    //std::vector<TableType> graph_;
    std::vector<int> head_;
    std::vector<bool> vis_;
};
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

    std::map<int, std::vector<Vec3i>> getLines() { return line_sets_; };



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
    std::map<int /*id*/, std::vector<Vec3i> /*line*/> line_sets_;

    std::shared_ptr<Graph> graph_;

};
}
#endif //SERIALPORT_BOUSTROPHEDON_H
