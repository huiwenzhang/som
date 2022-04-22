#pragma once

class XY {
public:
    XY();
    XY(double x, double y);
    ~XY();

    inline double x() const { return x_; }
    inline double y() const { return y_; }
    inline double& x() { return x_; }
    inline double& y() { return y_; }
    inline double& operator[](int i) {
        if (0 == i || i < 0)
            return x_;
        else
            return y_;
    }
    inline const double& operator[](int i) const {
        if (0 == i || i < 0)
            return x_;
        else
            return y_;
    }
    inline XY operator-() const { return XY(-x_, -y_); }
    inline XY operator-(const XY& arg) const { return XY(x_ - arg.x_, y_ - arg.y_); }
    inline XY operator+(const XY& arg) const { return XY(x_ + arg.x_, y_ + arg.y_); }

private:
    double x_;
    double y_;
};
