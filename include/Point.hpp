#ifndef POINT_H
#define POINT_H

namespace RHCL
{
    class Point
    {
    private:
        /* data */
        double _x, _y, _z;     //point's coordinate
        double _r, _g, _b, _a; //point's color
    public:
        Point(double x, double y, double z);
        Point(double x, double y, double z, double r, double g, double b, double a = 1.0f);
        ~Point();
    };

    /**
     * @brief Construct a new Point:: Point object
     * 
     * @param x 
     * @param y 
     * @param z 
     */
    Point::Point(double x, double y, double z) : _x(x), _y(y), _z(z)
    {
    }

    /**
     * @brief Construct a new Point:: Point object
     * 
     * @param x 
     * @param y 
     * @param z 
     * @param r 
     * @param g 
     * @param b 
     * @param a 
     */
    Point::Point(double x, double y, double z, double r, double g, double b, double a) : _x(x), _y(y), _z(z), _r(r), _g(g), _b(b), _a(a)
    {
    }

    /**
     * @brief Destroy the Point:: Point object
     * 
     */
    Point::~Point()
    {
    }

} // namespace RHCL

#endif //POINT_H