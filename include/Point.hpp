#ifndef POINT_H
#define POINT_H

namespace RHCL {
    class Point {
    private:
        /* data */
        double _x, _y, _z;     //point's coordinate
        double _r, _g, _b, _a; //point's color
    public:
        Point(/*args*/);

        Point(double x, double y, double z);

        Point(double x, double y, double z, double r, double g, double b, double a = 1.0f);

        ~Point();

        inline void setX(double x) { _x = x; } //set X

        inline void setY(double y) { _y = y; } // set Y

        inline void setZ(double z) { _z = z; } // set Z

        inline void setRGBA(double r, double g, double b, double a = 1.0f) {
            _r = r;
            _g = g;
            _b = b;
            _a = a;
        } // set RGBA
    };

} // namespace RHCL

#endif //POINT_H