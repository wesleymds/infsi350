#ifndef RAY_H
#define RAY_H

#include "Vec3.h"
#include "Mesh.h"
#include "utility"

#define v_of_tri(t, i) mesh.V[t.v[i]].p

class Ray {
private:
    float epsilon = 0.001;
    Mesh& mesh;
public:
    Vec3f origin;
    Vec3f direction;

    Ray(Mesh& mesh)
        : mesh(mesh)
    {}

    void set_ray(const Vec3f& that_origin, const Vec3f& that_direction) {
        origin = that_origin;
        direction = that_direction;
    }

    pair<bool, float> IntersectionRayonTriangle(const Triangle& tri) {
        Vec3f e0, e1, n, q, s, r;
        float a, b0, b1, b2, t;
        e0 = v_of_tri(tri, 1) - v_of_tri(tri, 0);
        e1 = v_of_tri(tri, 2) - v_of_tri(tri, 0);

        n = cross(e0, e1);
        n.normalize();
        q = cross(direction, e1);

        a = dot(e0, q);
        if ( dot(n, direction) >= 0 || abs(a) < epsilon) return pair<bool, float>(false, 0.f);

        s = (origin - v_of_tri(tri, 0))/a;
        r = cross(s, e0);
        b0 = dot(s, q);
        b1 = dot(r, direction);
        b2 = 1 - b0 - b1;
        if (b0 < 0 || b1 < 0 || b2 < 0) pair<bool, float>(false, 0.f);

        t = dot(e1, r);
        if (t >= 0)
            pair<bool, float>(true, t);
        pair<bool, float>(false, 0.f);
    }
};

#endif // RAY_H

