#ifndef ENGINE_H
#define ENGINE_H

#include "Vec3.h"
#include "Ray.h"
#include "Mesh.h"
#include "KDNode.h"
#include <chrono>
#include <ctime>
#include "Render.h"

const float Ray::epsilon(0.00000001);

class Engine {
private:
    float height;
    float width;
    float dx, dy, dx_2, dy_2;

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
    KDNode& node;
    static Vec3f lightPosRendu;
    static unsigned int pathNumber;
    static unsigned int pathLength;

    Engine(const float fovAngle,
           const float aspectRatio,
           const Vec3f& up,
           float& nearPlane,
           Vec3f& sceneCenter,
           const unsigned int screenWidth,
           const unsigned int screenHeight,
           Mesh& mesh,
           KDNode& node)
        :
          fovAngle(fovAngle),
          aspectRatio(aspectRatio),
          sceneCenter(sceneCenter),
          nearPlane(nearPlane),
          up(up),
          screenWidth(screenWidth),
          screenHeight(screenHeight),
          mesh(mesh),
          node(node)
    {}

    inline Vec3f getWorldCam(const Vec3f& camEyePolar) {
        Vec3f eye = polarToCartesian(camEyePolar);
        swap (eye[1], eye[2]);
        eye += sceneCenter;
        return eye;
    }

    void rayTrace(const Vec3f& camEyePolar, unsigned char* rayImage, unsigned int _w, unsigned int _h, bool useKDTree = true) {
        chrono::time_point<chrono::system_clock> start, end;
        start = chrono::system_clock::now();
        time_t startTime = chrono::system_clock::to_time_t(start);
        cout << "RayTrace start " << (useKDTree ? "with KDTree " : "") << ctime(&startTime);

        screenWidth = _w;
        screenHeight = _h;
        aspectRatio = screenWidth/(float)screenHeight;

        height = 2.f * tan(fovAngle * (M_PI / 360.f));
        width = height * aspectRatio;
        dx = width / screenWidth;
        dy = height / screenHeight;
        dx_2 = dx / 2.f;
        dy_2 = dy / 2.f;

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
        int ind(0);
        Vec3f colorResponse;

        Render render(mesh, node, pathLength, lightPosRendu);
        for (unsigned int i = 0; i < screenHeight; i++) {
            ind = 3*i*screenWidth;
            for (unsigned int j = 0; j < screenWidth; j++) {
                ind = 3*(j+i*screenWidth);
                location = l + u * j * dx + v * i * dy + u * dx_2 + v * dy_2;
                rayDir = location - eye;
                ray.setDirection(rayDir);

                colorResponse = Vec3f(0.f, 0.f, 0.f);

                for (unsigned int k = 0; k < pathNumber; k++)
                    colorResponse += render.tracePath(ray, 1, useKDTree);

                colorResponse *= 255.f/pathNumber;

                for(unsigned int k = 0; k < 3; k++) {
                    if (colorResponse[k] > 255.f) colorResponse[k] = 255.f;
                    rayImage[ind + k] = colorResponse[k];
                }
            }
            ind += 3;
        }

        // For anti-aliasing
        /*unsigned int x_rays, y_rays;
        float dxx, dyy;
        if (pathNumber == 1) {
            x_rays = 1;
            y_rays = 1;
            dxx = dx_2;
            dyy = dy_2;
        } else {
            x_rays = pathNumber/2;
            y_rays = pathNumber - x_rays;
            dxx = dx / x_rays;
            dyy = dy / y_rays;
        }

        for (unsigned int i = 0; i < screenHeight; i++) {
            ind = 3*i*screenWidth;
            for (unsigned int j = 0; j < screenWidth; j++) {
                ind = 3*(j+i*screenWidth);
                colorResponse = Vec3f(0.f, 0.f, 0.f);

                for (unsigned int x=0; x < x_rays; ++x) {
                    for (unsigned int y=0; y < y_rays; ++y) {

                    location = l + u * j * dx + v * i * dy + u * x * dxx + v * y * dyy;

                    rayDir = location - eye;
                    ray.setDirection(rayDir);

                    colorResponse += render.tracePath(ray, 1, useKDTree);
                    }
                }

                colorResponse *= 255.f/pathNumber;

                for(unsigned int k = 0; k < 3; k++) {
                    if (colorResponse[k] > 255.f) colorResponse[k] = 255.f;
                    rayImage[ind + k] = colorResponse[k];
                }
            }
            ind += 3;
        }*/

        end = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds = end-start;
        time_t endTime = chrono::system_clock::to_time_t(end);
        cout << "RayTrace finish " << (useKDTree ? "with KDTree " : "") << ctime(&endTime);
        cout << "Elapsed time: " << elapsed_seconds.count() << endl;
    }

};


#endif // ENGINE_h
