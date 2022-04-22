#include "tsp.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>

TspPlanner::TspPlanner(bool close, double dist_weight, double angle_weight)
        : close_loop_(close), dist_weight_(dist_weight), angle_weight_(angle_weight) {
    if (dist_weight_ == 0 && angle_weight_ == 0) {
        std::cout << "Both dist and angle weights are zero, set dist_weight = 1" << std::endl;
        dist_weight_ = 1.0;
    }
}

std::vector<Pnt3D> TspPlanner::normalize() {
    assert(locs_.size() > 0);
    Pnt3D min = locs_[0];
    Pnt3D max = locs_[0];
    // 找到最大值和最小值
    for (auto& loc : locs_) {
        if (loc.x() < min.x()) {
            min.x() = loc.x();
        } else if (loc.x() > max.x()) {
            max.x() = loc.x();
        }

        if (loc.y() < min.y()) {
            min.y() = loc.y();
        } else if (loc.y() > max.y()) {
            max.y() = loc.y();
        }

        if (loc.z() < min.z()) {
            min.z() = loc.z();
        } else if (loc.z() > max.z()) {
            max.z() = loc.z();
        }
    }

    double dx = max.x() - min.x();
    double dy = max.y() - min.y();
    double dz = max.z() - min.z();

    Pnt3D ratio(dx / dz, dy / dz, 1.0);
    ratio = ratio / ratio.max();
    // 归一化，并按照比例缩放
    std::vector<Pnt3D> normalized_city;
    for (auto pnt : locs_) {
        pnt = (pnt - min) / (max - min);
        pnt = pnt * ratio;
        normalized_city.push_back(pnt);
    }
    return normalized_city;
}

std::vector<Pnt3D> TspPlanner::generateNetwork() {
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<double> distr(0, 1);

    std::vector<Pnt3D> net(som_config_.population);
    for (int i = 0; i < som_config_.population; i++) {
        Pnt3D point(distr(engine), distr(engine), distr(engine));
        // minfo("neuron coord: %f %f %f", point.x(), point.y(), point.z());
        net.push_back(point);
    }
    return net;
}

int TspPlanner::getClosest(const std::vector<Pnt3D>& net, const Pnt3D& city) {
    assert(net.size() > 0);
    int closest = 0;
    double min_dist = net[0].getDist(city);
    for (int i = 1; i < net.size(); i++) {
        double dist = net[i].getDist(city);
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    return closest;
}

int TspPlanner::getClosestInRange(const std::vector<Pnt3D>& net, const Pnt3D& city) {
    assert(net.size() > 0);
    int closest = 0;
    struct Dist {
        size_t id;
        double distance;
    };
    std::vector<Dist> vec_dist;

    Dist d;
    double min_dist = net[0].getDist(city);

    d.id = 0;
    d.distance = min_dist;
    vec_dist.push_back(d);
    for (int i = 1; i < net.size(); i++) {
        double dist = net[i].getDist(city);
        d.id = i;
        d.distance = dist;
        vec_dist.push_back(d);
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    // minfo("Original best id %d", closest);
    if (angle_weight_ <= 1e-6) { return closest; }

    if (latest_visited_cities_.size() == 2) {
        Vec3D last(latest_visited_cities_.front().loc, latest_visited_cities_.back().loc);
        Vec3D now(latest_visited_cities_.back().loc, city);

        double angle = last.getAngle(now);
        double range = min_dist + angle_weight_ * angle / M_PI;

        int last_neuron = latest_visited_cities_.back().closest_neuron;
        int farest_id = 0;

        for (auto& ele : vec_dist) {
            // 在范围内，则返回和上一个neuron最远的
            int id_dist = ele.id - last_neuron;
            if (ele.distance > range || id_dist < 0) { continue; }

            if (id_dist > farest_id) {
                farest_id = id_dist;
                closest = ele.id;
            }
        }
    }

    return closest;
}

void TspPlanner::calcNeighborhood(int winner_idx,
                                  int radius,
                                  int size,
                                  std::vector<double>& weights) {
    if (radius < 1) { radius = 1; }
    for (int i = 0; i < size; i++) {
        double deltas = abs(winner_idx - i);
        if (close_loop_) { deltas = std::min(deltas, size - deltas); }
        double w = std::exp(-deltas * deltas / (2 * radius * radius));
        weights.push_back(w);
    }
}

void TspPlanner::calcRoute(const std::vector<Pnt3D>& net,
                           const std::vector<Pnt3D>& cities,
                           std::vector<Pnt3D>& path) {
    std::map<int, Pnt3D> map_path;
    for (auto loc : cities) {
        int index = getClosest(net, loc);
        map_path[index] = loc;  // map默认递增排序，获得路径顺序
    }
    for (auto it = map_path.begin(); it != map_path.end(); it++) { path.push_back(it->second); }
    if (close_loop_) { path.push_back(path.front()); }
}

double TspPlanner::calcRouteLen(const std::vector<Pnt3D>& cities) {
    if (cities.empty()) { return 0.0; }
    double sum = 0.0;
    for (int i = 0; i < cities.size() - 1; i++) { sum += cities[i].getDist(cities[i + 1]); }
    return sum;
}

double TspPlanner::calcRouteAngleLen(const std::vector<size_t>& cities) {
    if (cities.size() <= 2) { return 0.0; }
    double sum = 0.0;
    for (int i = 2; i < cities.size(); i++) {
        Vec3D v1(locs_[i - 2], locs_[i - 1]);
        Vec3D v2(locs_[i - 1], locs_[i]);
        sum += v1.getAngle(v2);
    }
    return sum;
}

void TspPlanner::runSOM(std::vector<Pnt3D>& path) {
    som_config_.population = locs_.size() * 8;
    som_config_.printSomConfig();

    std::vector<Pnt3D> normalized_city = normalize();
    std::vector<Pnt3D> net = generateNetwork();
    std::cout << "Network of " << som_config_.population << " neurons created, start iteration "
              << std::endl;

    srand(time(NULL));
    latest_visited_cities_.clear();
    for (int iter = 0; iter < som_config_.max_iters; iter++) {
        if (iter % 2000 == 0) {
            std::cout << "Iteration step " << iter << ", population size " << som_config_.population
                      << " , lr " << som_config_.lr << std::endl;
            saveResult("data/out_" + std::to_string(iter) + ".csv", net);
        }
        int idx = rand() % locs_.size();
        Pnt3D city = normalized_city[idx];
        int winner_idx = getClosestInRange(net, city);

        SldPoint latest;
        latest.id = idx;
        latest.closest_neuron = winner_idx;
        latest.loc = locs_[idx];
        latest_visited_cities_.push_back(latest);
        if (latest_visited_cities_.size() > 2) { latest_visited_cities_.pop_front(); }

        std::vector<double> weights;
        calcNeighborhood(winner_idx, int(som_config_.population / 10), net.size(), weights);
        int i = 0;
        for (auto& neuron : net) {
            neuron = neuron + weights[i] * som_config_.lr * (city - neuron);
            i++;
        }

        som_config_.lr *= som_config_.decay_factor;
        som_config_.population *= som_config_.decay_factor;

        if (som_config_.population < 1 || som_config_.lr < 0.001) {
            std::cout << "Radius or lr has completely decayed with lr = " << som_config_.lr
                      << ", population = " << som_config_.population << ", stop iteration"
                      << std::endl;
            break;
        }
    }
    calcRoute(net, normalized_city, path);  // 生成路径
    saveResult("data/final_path.csv", path);
    saveResult("data/normalized_cities.csv", normalized_city);

    double dist = calcRouteLen(path);
    std::cout << "Optimal path length: " << dist << std::endl;
}

void TspPlanner::readCityCoord(const std::string& filename) {
    using namespace std;
    ifstream readfile(filename, ios_base::in);
    if (!readfile) {
        std::cout << "Can't open file" << std::endl;
        return;
    }
    locs_.clear();
    while (!readfile.eof() && readfile.good()) {
        Pnt3D point;
        int id;
        readfile >> id >> point.x() >> point.y() >> point.z();
        locs_.push_back(point);
    }
    readfile.close();
    std::cout << "TspPlanner, num city " << locs_.size() << std::endl;
}

void TspPlanner::saveResult(const std::string& filename, const std::vector<Pnt3D>& path) const {
    std::ofstream out(filename, std::ofstream::trunc);

    for (auto& coord : path) { out << coord.x() << " " << coord.y() << " " << coord.z() << "\n"; }
    out.close();
}