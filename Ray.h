#ifndef RAY_H
#define RAY_H

#include <iostream>
#include "Vec3.h"
#include "tiny_obj_loader.h"

class Ray {
private:
    static float epsilon;

    Vec3f origin;
    Vec3f direction;
public:
    Ray() {}

    Ray(const Vec3f& origin, const Vec3f& direction)
        : origin(origin), direction(direction)
    {}

    int rayTriangleIntersection(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Vec3f& out) {
        Vec3f e0, e1, n, q, s, r;
        float a, b0, b1, b2, t;
        e0 = p1 - p0;
        e1 = p2 - p0;

        n = cross(e0, e1);
        n.normalize();
        q = cross(direction, e1);

        a = dot(e0, q);
        if ( dot(n, direction) >= 0 || abs(a) < epsilon) return 0;

        s = (origin - p0)/a;
        r = cross(s, e0);
        b0 = dot(s, q);
        b1 = dot(r, direction);
        b2 = 1 - b0 - b1;
        if (b0 < 0 || b1 < 0 || b2 < 0) return 0;

        t = dot(e1, r);
        if (t >= 0) {
            out = origin + t * direction;
            return 1;
        }
        return 0;
    }

    ///TODO
    int raySceneIntersection(const std::vector<tinyobj::shape_t>& scene) {
        return 0;
    }
};

#endif // RAY_H

