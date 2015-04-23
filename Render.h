#ifndef Render_H
#define Render_H

#include "Vec3.h"
#include "Mesh.h"
#include <algorithm>
#include <vector>
#include "Ray.h"

class Render {
public:
    Mesh& mesh;
	int maxDepthPath;
	Vec3f& lightPositionDirect;

    Render(Mesh& mesh, int maxDepthPath, Vec3f& lightPositionDirect)
        : mesh(mesh), maxDepthPath(maxDepthPath), lightPositionDirect(lightPositionDirect)
    {}

    // Compute the BRDF GGX model of light response in a vertice
    Vec3f BRDF_GGX (Vertex& v, Vec3f& camPosition, Vec3f& lightPosition) {
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

		// Incidence direction
		wi = lightPosition - v.p;
		wi.normalize();
		
		float vn_dot_wi = std::max(dot(vn, wi), 0.f);

        // Emission direction
        wo = camPosition - v.p;
		wo.normalize();

        // HalfVector
        wh = wi + wo;
        wh.normalize();

        // D(wi, wo): GGX distribution
        aux = 1 + (pow(alpha, 2.f) - 1.f) * pow(dot(vn, wh), 2.f);
        d = pow(alpha, 2.f) / (M_PI * pow(aux, 2.f));

        // F(wi, wh): Fernel term
        for(unsigned int i = 0; i < 3; ++i)
			f0[i] = mesh.material(v).specular[i];
		aux = std::max(dot(wi, wh), 0.f);
		
        for(unsigned int i = 0; i < 3; ++i)
			f[i] = f0[i] + (1.f - f0[i]) * pow((1.f - aux), 5.f);

        // G(wi, w0): Geometric term
        aux = pow(alpha, 2.f) + (1.f - pow(alpha, 2.f)) * pow(vn_dot_wi, 2.f);
        gi = (2 * dot(vn, wi)) / (dot(vn, wi) + sqrt(aux));
        aux = pow(alpha, 2.f) + (1.f - pow(alpha, 2.f)) * pow(dot(vn, wo), 2.f);
        go = (2 * dot(vn, wo)) / (dot(vn, wo) + sqrt(aux));
        g = gi * go;

		for(unsigned int i = 0; i < 3; ++i) {
			fs[i] = (d * f[i] * g) / (4.f * vn_dot_wi * dot(vn, wo)); // Specular term
			if (isinf(fs[i]))
				fs[i] = 1.f;
		}

        for(unsigned int i = 0; i < 3; ++i)
            fd[i] = mesh.material(v).diffuse[i] / M_PI; // Diffuse term

        // Final response
        res = (fd + fs) * vn_dot_wi;
        return res;
    }
	
	// Test if a a point of the scene (v) is occulted by any triangle
	bool isDirectedOcculted (Vertex& v) {
		
		// Light ray from v
		Vec3f direction = this->lightPositionDirect - v.p;
		direction.normalize();
		Ray lightRay(v.p, direction, mesh);

		Vertex v1, v2, v3, intersect;
		bool isIntersected = false;
		// Check intersection with all triangles of the mesh with lightRay
		for (unsigned int i = 0; i < mesh.T.size (); i++) {
			v1 = mesh.V[mesh.T[i].v[0]];
			v2 = mesh.V[mesh.T[i].v[1]];
			v3 = mesh.V[mesh.T[i].v[2]];
			isIntersected = lightRay.rayTriangleIntersection(v1.p, v2.p, v3.p, intersect);
			if (isIntersected && dist(intersect.p, v.p) < dist(this->lightPositionDirect, v.p))
				return true;
		}
		
		return false;
	}
	
	Vec3f evaluateResponse(Vertex& v, Vec3f& camPosition, Vec3f& lightPosition) {
				
		if (isDirectedOcculted(v))
			return Vec3f (0.f, 0.f, 0.f);
		else
			return BRDF_GGX(v, camPosition, lightPosition);
		
	}
	
	Vec3f tracePath(Ray& ray, int depth, Vec3f& camPosition) {

		Vertex v; // Vertex hit
		if (!(ray.raySceneIntersection(camPosition, v)))
			return Vec3f(0.f, 0.f, 0.f);  // Nothing was hit.
		
		// Compute the direct lightning
		Vec3f directedLight = evaluateResponse(v, camPosition, this->lightPositionDirect);
		
		if (depth == maxDepthPath)
			//return Vec3f(0.f, 0.f, 0.f);
			return directedLight; // Bounced enough times.
		
		// Pick a random direction from here and keep going
		Ray newRay(mesh, v.p);
		Vec3f newRayDirection = randomPointInHemisphereOf(v.n);
		newRay.setDirection(newRayDirection);
		
		// BRDF of the reflected response
		Vec3f BRDF = BRDF_GGX(v, camPosition, newRayDirection);
		
		// Evaluate le response for this new ray
		Vec3f reflectedLight = tracePath(newRay, depth + 1, v.p);
		
		return directedLight + reflectedLight * BRDF;
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
		
	float RandomFloat(float a, float b) {
		float random = ((float) rand()) / (float) RAND_MAX;
		float diff = b - a;
		float r = random * diff;
		return a + r;
	}
	



	
};

#endif // Render_H

