#ifndef ENGINE_H
#define ENGINE_H

#include "Vec3.h"
#include "Ray.h"
#include "Mesh.h"
#include "ResponseTrace.h"

const float Ray::epsilon(0.00000001);

class Engine {
private:
    float height;
    float width;
    float dx, dy;

    unsigned char* rayImage;

    Vec3f w, u, v, c, l;

public:
    const float fovAngle;
    float aspectRatio;
    Vec3f& sceneCenter;
    float& nearPlane;
    const Vec3f up;
    unsigned int screenWidth;
    unsigned int screenHeight;
    Mesh& mesh;
	static Vec3f lightPosRendu;


    Engine(const float fovAngle,
           const float aspectRatio,
           const Vec3f& up,
           float& nearPlane,
           Vec3f& sceneCenter,
           const unsigned int screenWidth,
           const unsigned int screenHeight,
           Mesh& mesh)
        :
          fovAngle(fovAngle),
          aspectRatio(aspectRatio),
          sceneCenter(sceneCenter),
          nearPlane(nearPlane),
          up(up),
          screenWidth(screenWidth),
          screenHeight(screenHeight),
          mesh(mesh)
    {}

    inline Vec3f getWorldCam(const Vec3f& camEyePolar) {
        Vec3f eye = polarToCartesian(camEyePolar);
        swap (eye[1], eye[2]);
        eye += sceneCenter;
        return eye;
    }

    void rayTrace(const Vec3f& camEyePolar, unsigned char* rayImage, unsigned int _w, unsigned int _h) {
        cout << "RayTrace start" << endl;

        screenWidth = _w;
        screenHeight = _h;
        aspectRatio = screenWidth/(float)screenHeight;

        //cout << screenWidth << " " << screenHeight << " " << aspectRatio << " " << fovAngle << endl;
        //cout << sceneCenter << endl;

        height = 2.f * tan(fovAngle * (M_PI / 360.f));
        width = height * aspectRatio;
        dx = width / screenWidth;
        dy = height / screenHeight;


        Vec3f eye(getWorldCam(camEyePolar));

        w = eye - sceneCenter;
        w.normalize();
        u = cross(up, w);
        u.normalize();
        v = cross(w, u);

		c = eye - w;

        l = c - (u * (width / 2.f)) - (v * (height / 2.f));
        Ray ray(mesh, eye);
        Vec3f rayDir, location;
        Vertex intersect;
        int ind(0);
		
		ResponseTrace responseTrace(mesh, lightPosRendu);

        for (unsigned int i = 0; i < screenHeight; ++i)
        {
            for (unsigned int j = 0; j < screenWidth; ++j)
            {
                ind = 3*(j+i*screenWidth);
                location = l + u * j * dx + v * i * dy + u * (dx / 2.f) + v * (dy / 2.f);
                rayDir = location - eye;
                ray.setDirection(rayDir);
                if (ray.raySceneIntersection(eye, intersect) == 1) {
                    const Vec3f colorResponse = responseTrace.evaluateResponse(intersect, eye);
                    for(auto i = 0; i < 3; ++i) rayImage[ind + i] = colorResponse[i];
                }
                else {
                    rayImage[ind] = rayImage[ind+1] = rayImage[ind+2] = 0;
                }
            }
        }

        cout << "RayTrace finish" << endl;
    }
};



#endif // ENGINE_h

/*cout << "RayTrace start" << endl;

float height, width, dx, dy;
Vec3f w, u, v, c, l;

cout << screenWidth << " " << screenHeight << " " << aspectRatio << " " << fovAngle << endl;

height = 2.f * nearPlane *  tan(fovAngle * (M_PI / 360.f));
width = height * aspectRatio;
dx = width / screenWidth;
dy = height / screenHeight;

Vec3f eye(getWorldCam(camEyePolar));

w = eye - sceneCenter;

w.normalize();
u = cross(up, w);
u.normalize();
v = cross(w, u);

c = eye - w * nearPlane;
l = c - (u * (width / 2.f)) - (v * (height / 2.f));
Ray ray(mesh, eye, sceneCenter);
Vec3f rayDir, location;
Vertex intersect;
int ind(0);

for (unsigned int i = 0; i < screenHeight; ++i)
{
    for (unsigned int j = 0; j < screenWidth; ++j)
    {
        ind = 3*(j+i*screenWidth);
        location = l + u * j * dx + v * i * dy + u * (dx / 2.f) + v * (dy / 2.f);
        rayDir = location - eye;
        ray.setDirection(rayDir);
        if (ray.raySceneIntersection(eye, intersect) == 1) {
            const Vec3f colorResponse = ray.evaluateResponse(intersect, eye);
            for(auto i = 0; i < 3; ++i) rayImage[ind + i] = colorResponse[i];
        }
        else {
            rayImage[ind] = rayImage[ind+1] = rayImage[ind+2] = 0;
        }
    }
}

cout << "RayTrace finish" << endl;*/
