

#pragma once
#include <list>
#include <vector>

#include "/pnt3d.h"
#include "/vec3d.h"

struct SldPoint {
    int id;    // 当前路径的sld索引
    int last;  // 上一个路径的索引
    Pnt3D loc;
    Vec3D normal;
    int closest_neuron;

    SldPoint(const Pnt3D& p, int index) : loc(p), id(index) {}
    SldPoint(const Pnt3D& p) : loc(p) {}
    SldPoint() = default;
};

class TspPlanner {
public:
    TspPlanner(bool close_loop, double dist_weight = 1.0, double angle_weight = 0.0);
    ~TspPlanner() {}

    struct SomConfig {
        double lr = 0.9;
        int max_iters = 1e5;
        double population;
        double decay_factor = 0.99997;

        void printSomConfig() const {
            // minfo("Som config, lr %f, max iters %d, decay factor %f, population %d",
            //       lr,
            //       max_iters,
            //       decay_factor,
            //       int(population));
        }
    };

    double calcRouteLen(const std::vector<Pnt3D>& cities);

    void runSOM(std::vector<Pnt3D>& path);  // SOM求解方法

    // som方法的超参数
    inline void setSomConfigs(const SomConfig& config) { som_config_ = config; }

    void readCityCoord(const std::string& filename);
    void saveResult(const std::string& filename, const std::vector<Pnt3D>& path) const;

private:
    std::vector<Pnt3D> normalize();
    std::vector<Pnt3D> generateNetwork();
    void calcNeighborhood(int winner_idx, int radius, int size, std::vector<double>& weights);
    /**
     * @brief 选择距离city delta范围内，距离上一个激活neuron最远的neuron
     *
     * @param network 神经元
     * @param city
     * @return int
     */
    int getClosestInRange(const std::vector<Pnt3D>& network, const Pnt3D& city);

    double calcRouteAngleLen(const std::vector<size_t>& cities);

    int getClosest(const std::vector<Pnt3D>& network, const Pnt3D& city);
    // void calcRoute(const std::vector<Pnt3D>& net, const std::vector<Pnt3D>& cities);
    void calcRoute(const std::vector<Pnt3D>& net,
                   const std::vector<Pnt3D>& cities,
                   std::vector<Pnt3D>& path);

private:
    std::vector<Pnt3D> locs_;  // 输入路径点坐标
    bool close_loop_;
    double dist_weight_;
    double angle_weight_;

    std::list<SldPoint> latest_visited_cities_;

    SomConfig som_config_;
};
