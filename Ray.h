#ifndef RAY_H
#define RAY_H

#include <iostream>
#include "Vec3.h"
#include "Mesh.h"
#include "tiny_obj_loader.h"

#define ivtri(i, tri) (scene.V[tri.v[i]].p)

using namespace std;

class Ray {
private:
    static const float epsilon;

    const Vec3f origin;
    Vec3f direction;
public:
    Ray() {}

    Ray(const Vec3f& origin, const Vec3f& direction)
        : origin(origin), direction(direction)
    {}

    Ray(const Vec3f& origin)
        : origin(origin)
    {}

    void setDirection(const Vec3f& _direction) {
        direction = _direction;
    }

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

    int raySceneIntersection(const Mesh& scene, Vec3f& eye, Vec3f& out) {
        Vec3f p0;
        Vec3f p1;
        Vec3f p2;
        int e(1000000), d;
        Vec3f intersect;
        bool isIntersect(false);
        for(auto tri: scene.T) {
            p0 = ivtri(0, tri);
            p1 = ivtri(1, tri);
            p2 = ivtri(2, tri);
            //cout << p0 << "|" << p1 << "|" << p2 << endl;
            if(rayTriangleIntersection(p0, p1, p2, intersect) == 1) {
                isIntersect = true;
                d = (eye - intersect).length();
                if(d < e) {
                    e = d;
                    out = intersect;
                }
            }
        }
        return isIntersect ? 1 : 0;
    }

    ///TODO
    void evaluateResponse() {

    }
};

#endif // RAY_H

