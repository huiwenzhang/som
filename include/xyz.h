#pragma once
#include <assert.h>

#include <cmath>

#include "xy.h"

class XYZ : public XY {
public:
    XYZ();
    XYZ(double x, double y, double z);
    ~XYZ();

    inline double z() const { return z_; }
    inline double& z() { return z_; }
    inline double& operator[](int i) {
        if (0 == i || i < 0)
            return XY::operator[](i);
        else if (1 == i)
            return XY::operator[](i);
        else
            return z_;
    }
    inline double operator[](int i) const {
        if (0 == i || i < 0)
            return XY::operator[](i);
        else if (1 == i)
            return XY::operator[](i);
        else
            return z_;
    }
    inline XYZ operator-() const { return XYZ(-x(), -y(), -z()); }
    inline XYZ operator-(const XYZ& arg) const {
        return XYZ(x() - arg.x(), y() - arg.y(), z() - arg.z());
    }
    inline XYZ operator+(const XYZ& arg) const {
        return XYZ(x() + arg.x(), y() + arg.y(), z() + arg.z());
    }
    inline XYZ operator*(double ratio) const { return XYZ(x() * ratio, y() * ratio, z() * ratio); }
    inline XYZ operator*(const XYZ& arg) const {
        return XYZ(x() * arg.x(), y() * arg.y(), z() * arg.z());
    }
    friend XYZ operator*(double ratio, const XYZ& data) { return data * ratio; }
    inline XYZ operator/(double ratio) const {
        assert(fabs(ratio) > 1e-6);
        return XYZ(x() / ratio, y() / ratio, z() / ratio);
    }
    inline XYZ operator/(const XYZ& arg) const {
        assert(fabs(arg.x()) > 1e-6);
        assert(fabs(arg.y()) > 1e-6);
        assert(fabs(arg.z()) > 1e-6);
        return XYZ(x() / arg.x(), y() / arg.y(), z() / arg.z());
    }
    inline bool operator!=(const XYZ& arg) const {
        return x() != arg.x() || y() != arg.y() || z() != arg.z();
    }

private:
    double z_;
};
