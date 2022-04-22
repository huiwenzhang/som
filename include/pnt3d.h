#pragma once

#include <math.h>

#include "xyz.h"

class Pnt3D : public XYZ {
public:
    Pnt3D();
    Pnt3D(const XYZ& xyz);
    Pnt3D(double x, double y, double z);
    ~Pnt3D();
    // return the distance between *this & p
    inline double getDist(const Pnt3D& p) const {
        double dx = this->x() - p.x();
        double dy = this->y() - p.y();
        double dz = this->z() - p.z();
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    // return the square of distance
    inline double getDist2(const Pnt3D& p) const {
        double dx = this->x() - p.x();
        double dy = this->y() - p.y();
        double dz = this->z() - p.z();
        return (dx * dx + dy * dy + dz * dz);
    }

    inline double max() {
        double a = this->x();
        if (a < this->y()) { a = this->y(); }
        if (a < this->z()) { a = this->z(); }
        return a;
    }

    inline double min() {
        double a = this->x();
        if (a > this->y()) { a = this->y(); }
        if (a > this->z()) { a = this->z(); }
        return a;
    }
};
