// ----------------------------------------------
// Informatique Graphique 3D & R�alit� Virtuelle.
// Projet
// Lancer de Rayon de Monte Carlo
// Copyright (C) 2015 Tamy Boubekeur
// All rights reserved.
// ----------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <GL/glut.h>
// Using for MacOS. Uncomment it.
//#include <GLUT/glut.h>
#include "Vec3.h"
#include "tiny_obj_loader.h"
#include "Ray.h"
#include "KDNode.h"
#include "Mesh.h"
#include "Engine.h"

using namespace std;

// App parameters
static const unsigned int DEFAULT_SCREENWIDTH = 1024;
static const unsigned int DEFAULT_SCREENHEIGHT = 768;
static const float DEFAULT_FOVANGLE = 45.f;
static const char * DEFAULT_SCENE_FILENAME = "scenes/cornell_box/cornell_box.obj";
static string appTitle ("MCRT - Monte Carlo Ray Tracer");
static GLint window;
static unsigned int screenWidth;
static unsigned int screenHeight;
static bool rayDisplayMode = false;
static unsigned int FPS = 0;
static bool KDTreeMode = false;
static bool boxMode = false;

// Camera parameters
static float fovAngle;
static float aspectRatio;
static float nearPlane;
static float farPlane;
static Vec3f camEyePolar; // Expressing the camera position in polar coordinate, in the frame of the target
static Vec3f camTarget;

// Scene elements

static Vec3f lightPos = Vec3f (1.f, 1.f, 1.f);
static Vec3f lightColor = Vec3f (1.f, 1.f, 1.f);
static Vec3f sceneCenter = Vec3f (0.f, 0.f, 0.f);
static float sceneRadius = 1.f;
static vector<tinyobj::shape_t> shapes;
static vector<tinyobj::material_t> materials;

// Mouse parameters
static bool mouseLeftButtonClicked = false;
static int clickedX, clickedY;
static float baseCamPhi;
static float baseCamTheta;

// Raytraced image
static unsigned char * rayImage = NULL;

//Engine settings
/*const float epsilon(0.00000001);
vector<KDNode*> tree;*/
const unsigned int Engine::numberRebonds = 1;
const Vec3f lightPosRendu(340.f, 500.f, 225.f);
KDNode node;
Mesh mesh;
Vec3f up(0.f, 1.f, 0.f);
Vec3f Engine::lightPosRendu = lightPosRendu;
Engine engine(DEFAULT_FOVANGLE,
              DEFAULT_SCREENWIDTH/(float)DEFAULT_SCREENHEIGHT,
              up,
              nearPlane,
              sceneCenter,
              DEFAULT_SCREENWIDTH,
              DEFAULT_SCREENHEIGHT,
              mesh,
              node
              );


void printUsage () {
    std::cerr << std::endl // send a line break to the standard error output
              << appTitle << std::endl
              << "Author : Tamy Boubekeur" << std::endl << std::endl
              << "Usage : ./myRayTracer [<file.obj>]" << std::endl
              << "Commandes clavier :" << std::endl
              << "------------------" << std::endl
              << " ?: Print help" << std::endl
              << " <space>: Toggle raytracing/rasterization (GL)  display mode" << std::endl
              << " r: Ray trace an image from the current point of view" << std::endl
              << " s: Save the current ray traced image under raytraced_image.ppm" << std::endl
              << " k: KDTree enable/disable" << std::endl
              << " <drag>+<left button>: rotate model" << std::endl
              << " <drag>+<right button>: move model" << std::endl
              << " <drag>+<middle button>: zoom" << std::endl
              << " q, <esc>: Quit" << std::endl << std::endl;
}

void initOpenGL () {
    glCullFace (GL_BACK);     // Specifies the faces to cull (here the ones pointing away from the camera)
    glEnable (GL_CULL_FACE); // Enables face culling (based on the orientation defined by the CW/CCW enumeration).
    glDepthFunc (GL_LESS); // Specify the depth test for the z-buffer
    glEnable (GL_DEPTH_TEST); // Enable the z-buffer in the rasterization
    glClearColor (0.0f, 0.0f, 0.0f, 1.0f); // Background color
    glEnable (GL_COLOR_MATERIAL);
}

void computeSceneNormals () {
    for (unsigned int s = 0; s < shapes.size (); s++)
        if (shapes[s].mesh.normals.empty ()) {
            shapes[s].mesh.normals.resize (shapes[s].mesh.positions.size (), 0.f);
            for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
                Vec3f q[3];
                for (size_t v = 0; v < 3; v++) {
                    unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                    for (unsigned int i = 0; i < 3; i++)
                        q[v][i] = shapes[s].mesh.positions[index+i];
                }
                Vec3f e01 = q[1] - q[0];
                Vec3f e02 = q[2] - q[0];
                Vec3f nf = normalize (cross (e01, e02));
                for (size_t v = 0; v < 3; v++) {
                    unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                    for (unsigned int i = 0; i < 3; i++)
                        shapes[s].mesh.normals[index+i] += nf[i];
                }
            }
            for (unsigned int i = 0; i < shapes[s].mesh.normals.size () / 3; i++) {
                Vec3f n;
                for (unsigned int j = 0; j < 3; j++)
                    n[j] = shapes[s].mesh.normals[3*i+j];
                n.normalize ();
                for (unsigned int j = 0; j < 3; j++)
                    shapes[s].mesh.normals[3*i+j] = n[j];
            }
        }
}

void computeSceneBoundingSphere () {
    sceneCenter = Vec3f (0.f, 0.f, 0.f);
    unsigned int count = 0;
    for (unsigned int s = 0; s < shapes.size (); s++)
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
            sceneCenter += Vec3f (shapes[s].mesh.positions[3*p],
                    shapes[s].mesh.positions[3*p+1],
                    shapes[s].mesh.positions[3*p+2]);
            count++;
        }
    sceneCenter /= count;
    sceneRadius = 0.f;
    for (unsigned int s = 0; s < shapes.size (); s++)
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
            float d = dist (sceneCenter, Vec3f (shapes[s].mesh.positions[3*p],
                            shapes[s].mesh.positions[3*p+1],
                    shapes[s].mesh.positions[3*p+2]));
            if (d > sceneRadius)
                sceneRadius = d;
        }
}

void initKDTree() {
    chrono::time_point<chrono::system_clock> start, end;
    start = chrono::system_clock::now();
    time_t startTime = chrono::system_clock::to_time_t(start);
    cout << "KDTreeConstruction start " << ctime(&startTime);

    std::vector<int> list;
    list.reserve(mesh.T.size());
    for (unsigned int i=0; i < mesh.T.size(); i++) list.push_back(i);

    node = *(KDNode::buildKDTree(mesh, list, 0.f));

    //KDNode::fastBuildKDTree(shapes, tree);

    end = chrono::system_clock::now();
    time_t endTime = chrono::system_clock::to_time_t(end);
    chrono::duration<double> elapsed_seconds = end-start;
    cout << "KDTreeConstrustion finish " << ctime(&endTime);
    cout << "Elapsed time: " << elapsed_seconds.count() << endl;
}

// Loads an OBJ file using tinyOBJ (http://syoyo.github.io/tinyobjloader/)
bool loadScene(const string & filename, const string & basepath = "") {
    chrono::time_point<chrono::system_clock> start, end;
    start = chrono::system_clock::now();
    time_t startTime = chrono::system_clock::to_time_t(start);
    cout << "Tiny scene construction start " << ctime(&startTime);

    shapes.clear ();
    materials.clear ();
    std::cout << "Loading " << filename << std::endl;
    std::string err = tinyobj::LoadObj(shapes, materials, filename.c_str (), basepath.c_str ());
    if (!err.empty()) {
        std::cerr << err << std::endl;
        return false;
    }
    computeSceneNormals ();
    computeSceneBoundingSphere ();

    end = chrono::system_clock::now();
    time_t endTime = chrono::system_clock::to_time_t(end);
    chrono::duration<double> elapsed_seconds = end-start;
    cout << "Tiny scene construction finish " << ctime(&endTime);
    cout << "Elapsed time: " << elapsed_seconds.count() << endl;

    start = chrono::system_clock::now();
    startTime = chrono::system_clock::to_time_t(start);
    cout << "Mesh scene construction start " << ctime(&startTime);

    engine.mesh.set_mesh(shapes, materials);
    engine.mesh.show_properties();

    end = chrono::system_clock::now();
    endTime = chrono::system_clock::to_time_t(end);
    elapsed_seconds = end-start;
    cout << "Mesh scene construction finish " << ctime(&endTime);
    cout << "Elapsed time: " << elapsed_seconds.count() << endl;

    initKDTree();
    return true;
}

void initCamera () {
    fovAngle = DEFAULT_FOVANGLE;
    nearPlane = sceneRadius/10000.0f;
    farPlane = 10*sceneRadius;
    camTarget = sceneCenter;
    camEyePolar = Vec3f (2.f * sceneRadius, M_PI/3.f, 1.5f * M_PI);
}

void initLighting () {
    lightPos = 2.f * Vec3f (sceneRadius, sceneRadius, sceneRadius);
    glEnable (GL_LIGHTING);
    GLfloat position[4] = {lightPos[0], lightPos[1], lightPos[2], 1.0f};
    GLfloat color[4] = {lightColor[0], lightColor[1], lightColor[2], 1.0f};
    glLightfv (GL_LIGHT0, GL_POSITION, position);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, color);
    glLightfv (GL_LIGHT0, GL_SPECULAR, color);
    glEnable (GL_LIGHT0);
}

void init (const string & filename) {  
    initOpenGL ();
    unsigned int i = filename.find_last_of ("/");
    loadScene (filename, filename.substr (0, i+1));
    initCamera ();
    initLighting ();
}

void setupCamera () {
    glMatrixMode (GL_PROJECTION); // Set the projection matrix as current. All upcoming matrix manipulations will affect it.
    glLoadIdentity ();
    gluPerspective (fovAngle, aspectRatio, nearPlane, farPlane); // Set the current projection matrix with the camera intrinsics
    glMatrixMode (GL_MODELVIEW); // Set the modelview matrix as current. All upcoming matrix manipulations will affect it.
    glLoadIdentity ();
    Vec3f eye = polarToCartesian (camEyePolar);
    swap (eye[1], eye[2]); // swap Y and Z to keep the Y vertical
    eye += camTarget;
    gluLookAt (eye[0], eye[1], eye[2],
            camTarget[0], camTarget[1], camTarget[2],
            0.0, 1.0, 0.0); // Set up the current modelview matrix with camera transform
}

void reshape (int w, int h) {
    screenWidth = w;
    screenHeight = h;
    aspectRatio = static_cast<float>(w)/static_cast<float>(h);
    glViewport (0, 0, (GLint)w, (GLint)h); // Dimension of the drawing region in the window
    setupCamera ();
    if (rayImage != NULL)
        delete [] rayImage;
    unsigned int l = 3*screenWidth*screenHeight;
    rayImage = new unsigned char [l];
    memset (rayImage, 0, l);
}

void rasterize () {
    setupCamera ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Erase the color and z buffers.
    glBegin (GL_TRIANGLES);
    glColor3f (1.f, 1.f, 1.f);
    for (size_t s = 0; s < shapes.size (); s++)
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
            if (!materials.empty ()) {
                unsigned int i = shapes[s].mesh.material_ids[f];
                glColor3f (materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
            }
            for (size_t v = 0; v < 3; v++) {
                unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                glNormal3f (shapes[s].mesh.normals[index],
                            shapes[s].mesh.normals[index+1],
                        shapes[s].mesh.normals[index+2]);
                glVertex3f (shapes[s].mesh.positions[index],
                            shapes[s].mesh.positions[index+1],
                        shapes[s].mesh.positions[index+2]);
            }
        }
    glEnd ();

    if (boxMode) {
        float length = (node.data.coins[1] - node.data.coins[0]).length() / sqrt(2);
        Vec3f center = (node.data.coins[1] + node.data.coins[0])/2.f;
        glLineWidth(5.f);
        glPushMatrix();
        glTranslated(sceneCenter[0], sceneCenter[1], sceneCenter[2]);
        glutWireCube(length);
        glPopMatrix();
    }

    glFlush (); // Ensures any previous OpenGL call has been executed
    glutSwapBuffers ();  // swap the render buffer and the displayed (screen) one
}

void displayRayImage () {
    glDisable (GL_DEPTH_TEST);
    glDrawPixels (screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, static_cast<void*>(rayImage));
    glutSwapBuffers ();
    glEnable (GL_DEPTH_TEST);
}

// MAIN FUNCTION TO CHANGE !
void rayTrace () {
    if (KDTreeMode) engine.rayTraceKDTree(camEyePolar, rayImage, screenWidth, screenHeight);
    else engine.rayTrace(camEyePolar, rayImage, screenWidth, screenHeight);
    //fastRayTrace();
}

void display () {
    if (rayDisplayMode)
        displayRayImage ();
    else rasterize ();
}

void saveRayImage (const string & filename) {
    if (rayImage != NULL) {
        std::ofstream out (filename.c_str ());
        out << "P3" << endl
            << screenWidth << " " << screenHeight << endl
            << "255" << endl;
        for (unsigned int i = 0; i < 3*screenWidth*screenHeight; i++)
            out << static_cast<int>(rayImage[i]) << " ";
        out.close ();
    }
}

void keyboard (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    //boxMode enable/disable
    case 'b':
        boxMode = !boxMode;
        glutPostRedisplay ();
        break;
    //KDtree enable/disable
    case 'k':
        KDTreeMode = !KDTreeMode;
        cout << "KDTReeMode " << (KDTreeMode ? "enable" : "disable") << endl;
        glutPostRedisplay ();
        break;
    case ' ':
        rayDisplayMode = !rayDisplayMode;
        glutPostRedisplay ();
        break;
    case 'r':
        rayTrace ();
        glutPostRedisplay ();
        break;
    case 's':
        saveRayImage ("raytraced_image.ppm");
        break;
    case 'q':
    case 27:
        exit (0);
        break;
    default:
        printUsage ();
        break;
    }
}

void mouse (int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseLeftButtonClicked = true;
            clickedX = x;
            clickedY = y;
            baseCamPhi = camEyePolar[1];
            baseCamTheta = camEyePolar[2];
        } else {
            mouseLeftButtonClicked = false;
        }
    }
}

void motion (int x, int y) {
    if (mouseLeftButtonClicked == true) {
        camEyePolar[1] = baseCamPhi + (float (clickedY-y)/screenHeight) * M_PI;
        camEyePolar[2] = baseCamTheta + (float (x-clickedX)/screenWidth) * M_PI;
        glutPostRedisplay (); // calls the display function
    }
}

// This function is executed in an infinite loop. 
void idle () {
    static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    static unsigned int counter = 0;
    counter++;
    float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    if (currentTime - lastTime >= 1000.0f) {
        FPS = counter;
        counter = 0;
        static char winTitle [128];
        unsigned int numOfTriangles = engine.mesh.T.size ();
        sprintf (winTitle, "Number Of Triangles: %d - FPS: %d", numOfTriangles, FPS);
        glutSetWindowTitle (winTitle);
        lastTime = currentTime;
    }
    glutPostRedisplay ();
}

int main (int argc, char ** argv) {
    glutInit (&argc, argv); // Initialize a glut app
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE); // Setup a RGBA framebuffer to display, with a depth buffer (z-buffer), in double buffer mode (fill a buffer then update the screen)
    glutInitWindowSize (DEFAULT_SCREENWIDTH, DEFAULT_SCREENHEIGHT); // Set the window app size on screen
    window = glutCreateWindow (appTitle.c_str ()); // create the window
    if (argc > 1)
        init (argv[1]); // Your initialization code (OpenGL states, geometry, material, lights, etc)
    else
        init (DEFAULT_SCENE_FILENAME);
    glutReshapeFunc (reshape); // Callback function executed whenever glut need to setup the projection matrix
    glutDisplayFunc (display); // Callback function executed when the window app need to be redrawn
    glutKeyboardFunc (keyboard); // Callback function executed when the keyboard is used
    glutMouseFunc (mouse); // Callback function executed when a mouse button is clicked
    glutMotionFunc (motion); // Callback function executed when the mouse move
    glutIdleFunc (idle); // Callback function executed continuously when no other event happens (good for background procesing or animation for instance).
    printUsage (); // By default, display the usage help of the program
    glutMainLoop ();
    return 0;
}

/*int fastRayTriangleIntersection(const Vec3f* tri, const Vec3f& origin,const Vec3f& direction, Vec3f* out) {
    const Vec3f& p0 = tri[0];
    const Vec3f& p1 = tri[1];
    const Vec3f& p2 = tri[2];

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
        out[0] = origin + t * direction;
        out[1] = n;
        return 1;
    }
    return 0;
}

int fastRaySceneIntersection(const Vec3f& origin, const Vec3f& direction, Vec3f* out, unsigned int& out_mat) {
    unsigned int e(1000000), index;
    static unsigned int sizeVec3f = sizeof(Vec3f);
    static Vec3f* tri = new Vec3f[sizeVec3f*3];
    static Vec3f* intersect = new Vec3f[sizeVec3f*2];
    float d;
    bool isIntersect(false);

    for (size_t s = 0; s < shapes.size (); s++)
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
            for (size_t v = 0; v < 3; v++) {
                index = 3*shapes[s].mesh.indices[3*f+v];
                tri[v][0] = shapes[s].mesh.positions[index];
                tri[v][1] = shapes[s].mesh.positions[index+1];
                tri[v][2] = shapes[s].mesh.positions[index+2];
            }
            if(fastRayTriangleIntersection(tri, origin, direction, intersect)) {
                isIntersect = true;
                d = (origin - intersect[0]).length();
                if(d < e) {
                    e = d;
                    memcpy(out, intersect, sizeVec3f*2);
                    out_mat = shapes[s].mesh.material_ids[f];
                }
            }
        }
    return isIntersect ? 1 : 0;
}

float* fastBRDF_GGX (Vec3f* intersect, unsigned int mat,const Vec3f& camPosition) {
    // Parameters in equations
    float d, alpha, aux, g, gi, go;
    Vec3f wi, vn, wo, wh;
    float fs[3], fd[3], f[3], f0[3];
    unsigned int float_s_3 = sizeof(float) * 3;
    static float* res = new float[float_s_3];
    const tinyobj::material_t material = materials[mat];

    // Alpha term (roughness)
    alpha = pow(material.shininess, -2.0);

    // Normal of V
    vn = intersect[1];
    vn.normalize();

    // Incidence direction
    wi = lightPosRendu - intersect[0];
    wi.normalize();

    float vn_dot_wi = max(dot(vn, wi), 0.f);

    // Emission direction
    wo = camPosition - intersect[0];
    wo.normalize();

    // HalfVector
    wh = wi + wo;
    wh.normalize();

    // D(wi, wo): GGX distribution
    float alpha_2 = pow(alpha, 2.f);
    aux = 1 + (alpha_2 - 1.f) * pow(dot(vn, wh), 2.f);
    d = alpha_2 / (M_PI * pow(aux, 2.f));

    // F(wi, wh): Fernel term
    for(unsigned int i = 0; i < 3; ++i)
        f0[i] = material.specular[i];
    aux = std::max(dot(wi, wh), 0.f);
    memcpy(f0, material.diffuse, float_s_3);

    for(unsigned int i = 0; i < 3; ++i)
        f[i] = f0[i] + (1.f - f0[i]) * pow((1.f - aux), 5.f);

    // G(wi, w0): Geometric term
    aux = alpha_2 + (1.f - alpha_2) * pow(vn_dot_wi, 2.f);
    gi = (2 * dot(vn, wi)) / (dot(vn, wi) + sqrt(aux));
    aux = alpha_2 + (1.f - alpha_2) * pow(dot(vn, wo), 2.f);
    go = (2 * dot(vn, wo)) / (dot(vn, wo) + sqrt(aux));
    g = gi * go;

    for(unsigned int i = 0; i < 3; ++i) {
        fs[i] = (d * f[i] * g) / (4.f * vn_dot_wi * dot(vn, wo)); // Specular term
        if (isinf(fs[i]))
            fs[i] = 1.f;
    }

    for(unsigned int i = 0; i < 3; ++i)
        fd[i] = material.diffuse[i] / M_PI; // Diffuse term

    // Final response
    for(unsigned int i = 0; i < 3; ++i) res[i] = (fd[i] + fs[i]) * vn_dot_wi;
    return res;
}

void fastEvaluateResponse(Vec3f* intersect, unsigned int mat, const Vec3f& camPosition, unsigned int* out) {
    float* response = fastBRDF_GGX(intersect, mat, camPosition);
    for(unsigned int i = 0; i < 3; ++i) out[i] = 255*response[i];

}

void fastRayTrace() {
    chrono::time_point<chrono::system_clock> start, end;
    start = chrono::system_clock::now();
    time_t startTime = chrono::system_clock::to_time_t(start);
    cout << "FastRayTrace start " << ctime(&startTime);

    float height, width, dx, dy, dx_2, dy_2;
    Vec3f w, u, v, c, l;

    height = 2.f * tan(fovAngle * (M_PI / 360.f));
    width = height * aspectRatio;
    dx = width / screenWidth;
    dy = height / screenHeight;
    dx_2 = dx/2.f;
    dy_2 = dy/2.f;

    Vec3f eye = polarToCartesian (camEyePolar);
    swap (eye[1], eye[2]); // swap Y and Z to keep the Y vertical
    eye += camTarget;

    w = eye - camTarget;
    w.normalize();
    u = cross(up, w);
    u.normalize();
    v = cross(w, u);

    c = eye - w;

    l = c - (u * (width / 2.f)) - (v * (height / 2.f));
    Vec3f rayDir, location;
    Vec3f* intersect = new Vec3f[sizeof(Vec3f) * 2];
    unsigned int* colorResponse = new unsigned int[sizeof(unsigned int) * 3];
    unsigned int ind(0), mat;

    for (unsigned int i = 0; i < screenHeight; ++i)
    {
        ind = 3 * i * screenWidth;
        /// TODO optimiser ca
        //location = l + v * i * dy + v * dy_2 + u * dx_2;
        for (unsigned int j = 0; j < screenWidth; ++j)
        {
            location = l + u * j * dx + v * i * dy + u * dx_2 + v * dy_2;
            rayDir = location - eye;
            if (fastRaySceneIntersection(eye, rayDir, intersect, mat)) {
                fastEvaluateResponse(intersect, mat, eye, colorResponse);
                for(auto k = 0; k < 3; ++k) rayImage[ind + k] = colorResponse[k];
            }
            else {
                rayImage[ind] = rayImage[ind+1] = rayImage[ind+2] = 0;
            }
            ind += 3;
            //location += (u * dx);
        }
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;
    time_t endTime = chrono::system_clock::to_time_t(end);
    cout << "FastRayTrace finish " << ctime(&endTime);
    cout << "Elapsed time: " << elapsed_seconds.count() << endl;
}*/
// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
