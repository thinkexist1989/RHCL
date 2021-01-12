//
// Created by think on 2020/12/8.
//

#include "Point.hpp"

namespace RHCL {

    /**
    * @brief Construct a new Point:: Point object
    *
    * @param x
    * @param y
    * @param z
    */
    Point::Point(double x, double y, double z) : _x(x), _y(y), _z(z), _r(0),
                                                 _g(0), _b(0), _a(1.0f) {
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
    Point::Point(double x, double y, double z, double r, double g, double b, double a) : _x(x), _y(y), _z(z), _r(r),
                                                                                         _g(g), _b(b), _a(a) {
    }

    /**
     * @brief Destroy the Point:: Point object
     *
     */
    Point::~Point() {
    }

    /**
     * @brief Construct a new Point:: Point object
     */
    Point::Point() : _x(0), _y(0), _z(0), _r(0),
                     _g(0), _b(0), _a(1.0f) {

    }

}