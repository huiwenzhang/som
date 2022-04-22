#pragma once

#include <math.h>

#include <iostream>

#include "xyz.h"


class Pnt3D;
class Vec3D : public XYZ {
public:
    Vec3D();
    Vec3D(const XYZ& xyz);
    Vec3D(double x, double y, double z);
    Vec3D(const Pnt3D& sp, const Pnt3D& ep);
    ~Vec3D();
    inline double getNorm() const { return sqrt(x() * x() + y() * y() + z() * z()); }
    inline double normalize() {
        double mod = getNorm();
        if (mod < 1e-10) return 0.0;

        for (int i = 0; i < 3; i++) this->operator[](i) /= mod;

        return mod;
    }

    Vec3D normalized() const {
        double mod = getNorm();
        return Vec3D(x() / mod, y() / mod, z() / mod);
    }

    inline Vec3D operator*(const Vec3D& v2) const { return this->cross(v2); }
    inline Vec3D operator*(double ratio) const {
        return Vec3D(x() * ratio, y() * ratio, z() * ratio);
    }
    friend Vec3D operator*(double ratio, const Vec3D& v2) { return v2 * ratio; }

    inline Vec3D operator+(const Vec3D& v2) const {
        return Vec3D(x() + v2.x(), y() + v2.y(), z() + v2.z());
    }
    inline Vec3D cross(const Vec3D& v2) const {
        double a = y() * v2[2] - z() * v2[1];
        double b = z() * v2[0] - x() * v2[2];
        double c = x() * v2[1] - y() * v2[0];
        return Vec3D(a, b, c);
    }
    inline double dot(const Vec3D& v2) const { return x() * v2[0] + y() * v2[1] + z() * v2[2]; }

    double getAngle(const Vec3D& v2) const;
    //向量插值从start_vec到end_vec，比例系数t[0.0,1.0]
    static Vec3D interpVec(const Vec3D& start_vec, const Vec3D& end_vec, double t);
};


