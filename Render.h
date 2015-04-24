#ifndef Render_H
#define Render_H

#include "Vec3.h"
#include "Mesh.h"
#include <algorithm>
#include <vector>
#include "Ray.h"

class Render {
private:
    float epsilon = 0.01;
public:
    Mesh& mesh;
    KDNode& node;
    int maxDepthPath;
    Vec3f& lightPositionDirect;

    Render(Mesh& mesh, KDNode& node, int maxDepthPath, Vec3f& lightPositionDirect)
        : mesh(mesh), node(node), maxDepthPath(maxDepthPath), lightPositionDirect(lightPositionDirect)
    {}

    // Compute the BRDF GGX model of light response in a vertice
    Vec3f BRDF_GGX (Vertex& v, Vec3f& camPosition, Vec3f& lightDirection) {
        // Parameters in equations
        float d, roughness, alpha, aux, g, gi, go;
        Vec3f wi, vn, wo, wh;
        Vec3f fs, fd, f, f0, res;

        // Alpha term (roughness)
        roughness = sqrt(2.f / (2.f + mesh.material(v).shininess));
        alpha = pow(roughness, 2.f);

        // Normal of V
        vn = v.n;
        vn.normalize();

        // Incidence direction
        wi = lightDirection;
        wi.normalize();

        // Emission direction
        wo = camPosition - v.p;
        wo.normalize();

        // HalfVector
        wh = wi + wo;
        wh.normalize();

        float a_2 = pow(alpha, 2.f);

        // D(wi, wo): GGX distribution
        aux = 1 + (a_2 - 1.f) * pow(dot(vn, wh), 2.f);
        d = a_2 / (M_PI * pow(aux, 2.f));

        // F(wi, wh): Fernel term
        for(unsigned int i = 0; i < 3; ++i)
            f0[i] = mesh.material(v).specular[i];
        aux = std::max(dot(wi, wh), 0.f);

        for(unsigned int i = 0; i < 3; ++i)
            f[i] = f0[i] + (1.f - f0[i]) * pow((1.f - aux), 5.f);

        float vnwi = dot(vn, wi);
        float vnwo = dot(vn, wo);

        // G(wi, w0): Geometric term
        aux = a_2 + (1.f - a_2) * pow(vnwi, 2.f);
        gi = (2.f * abs(vnwi)) / (abs(vnwi) + sqrt(aux));
        aux = a_2 + (1.f - a_2) * pow(vnwo, 2.f);
        go = (2.f * abs(vnwo)) / (abs(vnwo) + sqrt(aux));
        g = gi * go;

        for(unsigned int i = 0; i < 3; ++i) {
            fs[i] = (d * f[i] * g) / (4.f * abs(vnwi) * abs(vnwo)); // Specular term
        }

        for(unsigned int i = 0; i < 3; ++i)
            fd[i] = mesh.material(v).diffuse[i] / M_PI; // Diffuse term

        // Final response
        res = (fd + fs) * (vnwi <  0.f ? 0.f : vnwi);

        return 2.f * res;
    }

    // Test if a a point of the scene (v) is occulted by any triangle
    bool isDirectedOcculted (Vertex& v, bool useKDTree = true) {
        // Light ray from v
        Vec3f direction = this->lightPositionDirect - v.p;
        direction.normalize();
        Ray lightRay(v.p, direction, mesh);
        Vertex intersect;
        if (useKDTree) {
            return lightRay.rayKDIntersection(&node, intersect, this->lightPositionDirect, true);
        } else {
            return lightRay.raySceneIntersection(intersect, this->lightPositionDirect, true);
        }
        return false;
    }

    Vec3f evaluateResponse(Vertex& v, Vec3f& camPosition, Vec3f& lightPosition, bool useKDTree = true) {
        if (isDirectedOcculted(v, useKDTree))
            return Vec3f (0.f, 0.f, 0.f);
        else {
            Vec3f lightDirection = lightPosition - v.p;
            return BRDF_GGX(v, camPosition, lightDirection);
        }
    }

    Vec3f tracePath(Ray& ray, int depth, bool useKDTree = true) {
        Vertex v; // Vertex hit
        const bool isIntersected = useKDTree ? ray.rayKDIntersection(&node, v, this->lightPositionDirect) : ray.raySceneIntersection(v, this->lightPositionDirect);
        if (!isIntersected) return Vec3f(0.f, 0.f, 0.f);  // Nothing was hit.

        // Compute the direct lightning
        Vec3f directedLight = evaluateResponse(v, ray.origin, this->lightPositionDirect, useKDTree);

        if (depth == maxDepthPath)
            return directedLight; // Bounced enough times.

        // Pick a random direction from here and keep going
        Ray newRay(mesh, v.p);
        Vec3f newRayDirection;

        newRayDirection = randomPointCosinusInHemisphereOf(v.n);
        newRay.setDirection(newRayDirection);

        // BRDF of the reflected response
        Vec3f BRDF = BRDF_GGX(v, ray.origin, newRayDirection);

        // Evaluate le response for this new ray
        Vec3f reflectedLight = tracePath(newRay, depth + 1);

        Vec3f res = directedLight + (reflectedLight * BRDF);

        return res;
    }

    // Generate random points in the hemisphere aligned with n
    Vec3f randomPointInHemisphereOf(Vec3f& n) {
        float x, y, z, d;
        do {
            x = RandomFloat(-1, 1);
            y = RandomFloat(-1, 1);
            z = RandomFloat(-1, 1);
            d = sqrt(pow(x, 2.f) + pow(y, 2.f) + pow(z, 2.f));
        } while(d > 1);

        Vec3f v(x, y, z);
        v.normalize();
        if (dot(v, n) < 0)
            v *= -1;
        return v;
    }

    inline float RandomFloat(float a, float b) {
        float random = ((float) rand()) / (float) RAND_MAX;
        float diff = b - a;
        float r = random * diff;
        return a + r;
    }

    // Generate random points weighted by cosinus
    Vec3f randomPointCosinusInHemisphereOf(Vec3f& n) {

        float a = RandomFloat(0.f, 1.f);

        float b = RandomFloat(0.f, 1.f);


        float r = sqrt(a);

        float theta = 2.f * M_PI * b;


        float x = r * cos(theta);

        float z = r * sin(theta);


        Vec3f v(x, sqrt(max(0.0f, 1.f - a)), z);

        if (dot(v, n) < 0)

            v *= -1;

        return v;

    }

};

#endif // Render_H

