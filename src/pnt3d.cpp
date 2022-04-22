#include "pnt3d.h"

Pnt3D::Pnt3D() {}

Pnt3D::Pnt3D(const XYZ& xyz) : XYZ(xyz.x(), xyz.y(), xyz.z()) {}

Pnt3D::Pnt3D(double x, double y, double z) : XYZ(x, y, z) {}

Pnt3D::~Pnt3D() {}
