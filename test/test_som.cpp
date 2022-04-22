#include "tsp.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Please input .tsp file name\n";
        return 1;
    }
    TspPlanner planner(false);

    planner.readCityCoord(argv[1]);

    std::vector<Pnt3D> path;
    planner.runSOM(path);

    return 0;
}