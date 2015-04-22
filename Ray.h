#ifndef RAY_H
#define RAY_H

#include <iostream>
#include "Vec3.h"
#include "Mesh.h"
#include "tiny_obj_loader.h"
#include "Brdf.h"

#define ivtri(i, tri) (mesh.V[tri.v[i]].p)

using namespace std;

class Ray {
private:

    static const float epsilon;

public:

    Vec3f origin;
    Vec3f direction;
    Ray() {}
    const Vec3f origin;
    Vec3f direction;
    const Vec3f& sceneCenter;

    Mesh& mesh;
public:
    Ray(const Vec3f& origin, const Vec3f& direction, Mesh& mesh, const Vec3f& sceneCenter)
        : origin(origin), direction(direction), mesh(mesh), sceneCenter(sceneCenter)
    {}

    Ray(Mesh& mesh, const Vec3f& origin, const Vec3f& sceneCenter)
        : origin(origin), mesh(mesh), sceneCenter(sceneCenter)
    {}

    void setDirection(const Vec3f& _direction) {
        direction = _direction;
    }

    int rayTriangleIntersection(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, Vertex& out) {
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
            out.p = origin + t * direction;
            out.n = n;
            return 1;
        }
        return 0;
    }

    int raySceneIntersection(const Vec3f& eye, Vertex& out) {
        Vec3f p0;
        Vec3f p1;
        Vec3f p2;
        int e(1000000), d;
        Vertex intersect;
        bool isIntersect(false);
        for(auto tri: mesh.T) {
            p0 = ivtri(0, tri);
            p1 = ivtri(1, tri);
            p2 = ivtri(2, tri);
            if(rayTriangleIntersection(p0, p1, p2, intersect) == 1) {
                isIntersect = true;
                d = (eye - intersect.p).length();
                if(d < e) {
                    e = d;
                    out = intersect;
                    out.material_id = tri.material_id;
                }
            }
        }
        return isIntersect ? 1 : 0;
    }



    Vec3f evaluateResponse(const Vertex& intersect, const Vec3f& camEye) {
		Vec3f lightPos = Vec3f (340.f, 450.f, 225.f);
		// Light ray from v
		Ray lightRay(intersect.p, lightPos, mesh, sceneCenter);
//
//		Vertex v;
//		float epsilon = 0.5;
//		// Check it there is an intersection between lightRay and a point of the scene
//		if (lightRay.raySceneIntersection(camEye, v))
//			// If the intersection is in a distance < epsilon
//			if (dist(v.p, intersect.p) > epsilon)
//				return 0;
		
		static Brdf brdf(mesh, sceneCenter);
		return brdf.reponseBRDF_GGX(intersect, camEye);
    }

};

#endif // RAY_H

