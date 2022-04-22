#include "xyz.h"

XYZ::XYZ() { z_ = 0.0; }

XYZ::XYZ(double x, double y, double z) : XY(x, y), z_(z) {}

XYZ::~XYZ() {}