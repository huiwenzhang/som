#include "vec3d.h"

#include "pnt3d.h"


Vec3D::Vec3D() {}

Vec3D::Vec3D(const XYZ& xyz) : XYZ(xyz.x(), xyz.y(), xyz.z()) {}

Vec3D::Vec3D(double x, double y, double z) : XYZ(x, y, z) {}

Vec3D::Vec3D(const Pnt3D& sp, const Pnt3D& ep) {
    this->x() = ep.x() - sp.x();
    this->y() = ep.y() - sp.y();
    this->z() = ep.z() - sp.z();
}

Vec3D::~Vec3D() {}
double Vec3D::getAngle(const Vec3D& v2) const {
    double dist = this->dot(v2);
    double m1 = getNorm();
    double m2 = v2.getNorm();
    if (m1 < 1e-6 || m2 < 1e-6) return 0.0;
    double d = dist / (m1 * m2);
    d = d > 1 ? 1 : d < -1 ? -1 : d;
    return acos(d);
}

Vec3D Vec3D::interpVec(const Vec3D& start_vec, const Vec3D& end_vec, double t) {
    double angle = start_vec.getAngle(end_vec);
    if (t < 1e-2 || angle < 1e-2 || fabs(angle - 3.141592653589397) < 1e-2)
        return start_vec.normalized();
    else if (1 - t < 1e-2)
        return end_vec.normalized();
    else {
        double ag1 = angle * t;
        double ag2 = angle - ag1;
        double time = sin(ag1) / sin(ag2);
        return (start_vec.normalized() + end_vec.normalized() * time).normalized();
    }
    return Vec3D();
}
