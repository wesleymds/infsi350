#ifndef ENGINE_H
#define ENGINE_H

#include "Vec3.h"
#include "Ray.h"
#include "Mesh.h"
#include "Engine.h"

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
    const float aspectRatio;
    Vec3f& sceneCenter;
    float& nearPlane;
    const Vec3f up;
    const unsigned int screenWidth;
    const unsigned int screenHeight;
    Mesh& mesh;

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


    void rayTrace(const Vec3f& camEyePolar, unsigned char* rayImage) {
        cout << "RayTrace start" << endl;

        height = (2.f * nearPlane * tan(fovAngle / 2.f));
        //nearPlane == 1;
        //height(2.f * tan(fovAngle / 2.f)),
        width = height * aspectRatio;
        dx = width / (screenWidth - 1);
        dy = height / (screenHeight - 1);

        Vec3f eye(getWorldCam(camEyePolar));

        w = eye - sceneCenter;
        w.normalize();
        u = cross(up, w);
        u.normalize();
        v = cross(w, u);

        c = eye - w * nearPlane;
        //nearPlane == 1
        //c = eye - w;
        l = c - (u * (width / 2.f)) - (v * (height / 2.f));
        Ray ray(mesh, eye, sceneCenter);
        Vec3f rayDir, location;
        Vertex intersect;
        int ind(0);

        cout << nearPlane << endl;

        for (unsigned int i = 0; i < screenHeight; ++i)
        {
            for (unsigned int j = 0; j < screenWidth; ++j)
            {
                location = l + u * j * dx + v * i * dy; /*+ u * (dx / 2.f) + v * (dy / 2.f)*/;
                rayDir = location - eye;
                ray.setDirection(rayDir);
                ind = 3*(j+i*screenWidth);
                if (ray.raySceneIntersection(eye, intersect) == 1) {
                    /*rayImage[ind+2] = 255;
                    rayImage[ind] = rayImage[ind+1] = 255 / (eye-intersect).length();*/
                    const Vec3f colorResponse = ray.evaluateResponse(intersect, eye);
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

