#ifndef BRDF_H
#define BRDF_H

#include "Vec3.h"
#include "Mesh.h"

class Brdf {
public:
    Mesh& mesh;

    Brdf(Mesh& mesh)
        : mesh(mesh)
    {}

    // Compute the BRDF GGX model of light response in a vertice
    Vec3f reponseBRDF_GGX (const Vertex& v) {
        // Parameters in equations
        float d, roughness, alpha, aux, f, g, f0, gi, go;
        Vec3f wi, vn, wo, wh;

        // Alpha term (roughness)
        /*roughness = 0.5; // replace by 1/materials[i].shininess
    alpha = pow(roughness, 2.0);*/

        // Normal of V
        vn = v.n;
        vn.normalize();

        Vec3f lightPos = Vec3f (340.f, 500.f, 225.f);
        // Incident light
        wi = lightPos - v.p;
        wi.normalize();

        /*// Camera
    wo = polarToCartesian (camEyePolar);

    // HalfVector
    wh = wi + wo;
    wh.normalize();

    // D(wi, wo): GGX distribution
    aux = 1 + (pow(alpha, 2.0) - 1.0) * pow(dot(vn, wh), 2.0);
    d = pow(alpha, 2.0) / (M_PI * pow(aux, 2.0));

    // F(wi, wh): Fernel term
    f0 = 0.91; // replace by the reflexion coef materials[i].?
    aux = dot(wi, wh);
    if (aux < 0)
        aux = 0;
    f = f0 + (1 - f0) * pow((1 - aux), 5.0);

    // G(wi, w0): Geometric term
    aux = pow(alpha, 2.0) + (1 - pow(alpha, 2.0) * pow(dot(vn, wi), 2.0));
    gi = (2 * dot(vn, wi)) / (dot(vn, wi) + sqrt(aux));
    aux = pow(alpha, 2.0) + (1 - pow(alpha, 2.0) * pow(dot(vn, wo), 2.0));
    go = (2 * dot(vn, wo)) / (dot(vn, wo) + sqrt(aux));
    g = gi * go;*/

        //fs = (d * f * g) / (4 * dot(vn, wi) * dot(vn, wo)); // Specular term
        Vec3f fd;

        for(auto i = 0; i < 3; ++i)
            fd[i] = 255 * mesh.material(v).diffuse[i] / M_PI; // Diffuse term
        float fs = 0.f;

        Vec3f res;

        // Final response
        res = 1.f * (fd /*+ fs*/) * dot(vn, wi);

        return res;
    }
};

#endif // BRDF_H

