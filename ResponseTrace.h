#ifndef ResponseTrace_H
#define ResponseTrace_H

#include "Vec3.h"
#include "Mesh.h"
#include <algorithm>
#include <vector>
#include "Ray.h"

class ResponseTrace {
public:
    Mesh& mesh;
	Vec3f& lightPos;

    ResponseTrace(Mesh& mesh, Vec3f& lightPos)
        : mesh(mesh), lightPos(lightPos)
    {}

    // Compute the BRDF GGX model of light response in a vertice
    Vec3f responseBRDF_GGX (const Vertex& v, const Vec3f& eye) {
        // Parameters in equations
        float d, roughness, alpha, aux, g, gi, go;
        Vec3f wi, vn, wo, wh;
        Vec3f fs, fd, f, f0, res;

        // Alpha term (roughness)
        roughness = 1.f / mesh.material(v).shininess;
        alpha = pow(roughness, 2.0);

        // Normal of V
        vn = v.n;
        vn.normalize();

        // Incident light
        wi = lightPos - v.p;
        //wi = v.p - lightPos;
        wi.normalize();
		
		float vnDotwi = std::max(dot(vn, wi), 0.f);

        // Camera
        wo = eye;

        // HalfVector
        wh = wi + wo;
        wh.normalize();

        // D(wi, wo): GGX distribution
        aux = 1 + (pow(alpha, 2.0) - 1.0) * pow(dot(vn, wh), 2.0);
        d = pow(alpha, 2.0) / (M_PI * pow(aux, 2.0));

        // F(wi, wh): Fernel term
        for(auto i = 0; i < 3; ++i) f0[i] = mesh.material(v).specular[i];
		aux = std::max(dot(wi, wh), 0.f);
		
        for(auto i = 0; i < 3; ++i) f[i] = f0[i] + (1 - f0[i]) * pow((1 - aux), 5.0);

        // G(wi, w0): Geometric term
        aux = pow(alpha, 2.0) + (1 - pow(alpha, 2.0)) * pow(vnDotwi, 2.0);
        gi = (2 * dot(vn, wi)) / (dot(vn, wi) + sqrt(aux));
        aux = pow(alpha, 2.0) + (1 - pow(alpha, 2.0)) * pow(dot(vn, wo), 2.0);
        go = (2 * dot(vn, wo)) / (dot(vn, wo) + sqrt(aux));
        g = gi * go;

        for(auto i = 0; i < 3; ++i) fs[i] = (d * f[i] * g) / (4 * vnDotwi * dot(vn, wo)); // Specular term

        for(auto i = 0; i < 3; ++i)
            fd[i] = 255 * mesh.material(v).diffuse[i] / M_PI; // Diffuse term

        // Final response
        // TODO maybe puissance?
        //res = 1.f * (fd + fs) * dot(vn, wi);
        res = (fd + fs) * vnDotwi;
//        res = fd * vnDotwi;
//		res = Vec3f(255.f, 255.f, 255.f) * std::max(dot(vn, wi), 0.f);

        return res;
    }
	
	// Test if a a point of the scene (v) is occulted by any triangle
	// in a epsilon interval
	bool isDirectedOcculted (const Vertex& v) {
		
		// Light ray from v
		Vec3f direction = lightPos - v.p;
//		if (v.shapeName == "green_wall" && v.p[0] == 0 && v.p[1] > 400 && v.p[2] > 300) // test
//			cout << "test" << endl;
		direction.normalize();
		Ray lightRay(v.p, direction, mesh);

		
		Vertex v1, v2, v3, intersect;
		bool isIntersected = false;
		// Check intersection with all triangles of the mesh with lightRay
		for (unsigned int i = 0; i < mesh.T.size (); i++) {
			v1 = mesh.V[mesh.T[i].v[0]];
			v2 = mesh.V[mesh.T[i].v[1]];
			v3 = mesh.V[mesh.T[i].v[2]];
//			Triangle t = mesh.T[i]; // test
			isIntersected = lightRay.rayTriangleIntersection(v1.p, v2.p, v3.p, intersect);
			if (isIntersected && dist(intersect.p, v.p) < dist(lightPos, v.p))
				return true;
		}
		
		return false;
	}

	
	Vec3f evaluateResponse(const Vertex& v, const Vec3f& camEye) {
		
		if (isDirectedOcculted(v))
			return Vec3f (0.f, 0.f, 0.f);
		else
			return responseBRDF_GGX(v, camEye);
	}

	
};

#endif // ResponseTrace_H

