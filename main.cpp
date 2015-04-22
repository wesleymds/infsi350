// ----------------------------------------------
// Informatique Graphique 3D & Réalité Virtuelle.
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

using namespace std;

// App parameters
static const unsigned int DEFAULT_SCREENWIDTH = 1024;
static const unsigned int DEFAULT_SCREENHEIGHT = 768;
static const char * DEFAULT_SCENE_FILENAME = "scenes/cube/cube.obj";
static string appTitle ("MCRT - Monte Carlo Ray Tracer");
static GLint window;
static unsigned int screenWidth;
static unsigned int screenHeight;
static bool rayDisplayMode = false;
static unsigned int FPS = 0;

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

static Mesh mesh;

// Mouse parameters
static bool mouseLeftButtonClicked = false;
static int clickedX, clickedY;
static float baseCamPhi;
static float baseCamTheta;

// Raytraced image
static unsigned char * rayImage = NULL;

const float Ray::epsilon(0.001);

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

// Loads an OBJ file using tinyOBJ (http://syoyo.github.io/tinyobjloader/)
bool loadScene(const string & filename, const string & basepath = "") {
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

    mesh.set_mesh(shapes, materials);
    mesh.show_properties();
    return true;
}

void initCamera () {
    fovAngle = 135.f;
    nearPlane = sceneRadius/10000.0f;
    farPlane = 10*sceneRadius;
    camTarget = sceneCenter;
    camEyePolar = Vec3f (2.f * sceneRadius, M_PI/2.f, M_PI/2.f);
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
    glFlush (); // Ensures any previous OpenGL call has been executed
    glutSwapBuffers ();  // swap the render buffer and the displayed (screen) one
}

void displayRayImage () {
    glDisable (GL_DEPTH_TEST);
    glDrawPixels (screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, static_cast<void*>(rayImage));
    glutSwapBuffers ();
    glEnable (GL_DEPTH_TEST);
}

// Test if a a point of the scene (v) is occulted by any triangle
// in a epsilon interval
bool isDirectedOcculted (Vertex v, float epsilon) {
	// Light ray from v
	Ray lightRay(v.p, lightPos);

    Vec3f intersecT;
	// Check it there is an intersection between lightRay and a point of the scene
	if (lightRay.raySceneIntersection(mesh, intersecT))
		// If the intersection is in a distance < epsilon
		if (dist(intersecT, v.p) < epsilon)
			return true;
	
	return false;
}

// Compute the BRDF GGX model of light response in a vertice
float reponseBRDF_GGX (Vertex v) {
	// Parameters in equations
	float res, fd, fs, d, roughness, alpha, aux, f, g, f0, gi, go;
	Vec3f wi, vn, wo, wh;
	
	// Alpha term (roughness)
	roughness = 0.5; // replace by 1/materials[i].shininess
	alpha = pow(roughness, 2.0);
	
	// Normal of V
	vn = v.n;
	vn.normalize();
	
	// Incident light
	wi = lightPos - v.p;
	wi.normalize();
	
	// Camera
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
	g = gi * go;
	
	fd = 1.0 / M_PI; // Diffuse term
	fs = (d * f * g) / (4 * dot(vn, wi) * dot(vn, wo)); // Specular term
	
	// Final response
	res = 1 * (fd + fs) * dot(vn, wi);
	
	return res;
}

// MAIN FUNCTION TO CHANGE !
void rayTrace () {    
    Vec3f eye = polarToCartesian(camEyePolar);
    cout << eye << endl;
    Vec3f w = sceneCenter - eye;
    w.normalize();
    Vec3f b(0.f, 1.f, 0.f);
    Vec3f u = cross(b, w);
    u.normalize();
    Vec3f v = cross(w, u);
    v.normalize();
    float fovH = 2.f * atan(tan((fovAngle * M_PI) / 360.f) * aspectRatio);
    float alpha, beta;
    Vec3f add, rayDir;
    Ray ray(eye);
    Vec3f intersect;
    unsigned int ind;
    cout << "u=" << u << endl;
    cout << "w=" << w << endl;
    cout << "v=" << v << endl;

    ///TODO ray trace http://www1.cs.columbia.edu/~cs4162/slides/lecture16.pdf
    for (unsigned int  i = 0; i < screenHeight; i++) {
        beta = tan(fovAngle/2.f) * (((screenHeight/2.f) - i)/(screenHeight/2.f));
        for (unsigned int j = 0; j < screenWidth; j++) {
            alpha = tan(fovH/2.f) * ((j - (screenWidth/2.f))/(screenWidth/2.f));
            add = u * alpha + v * beta - w;
            //add.normalize();
            rayDir = add;
            rayDir.normalize();
            //cout << "raDIr="<< rayDir << endl;
            ray.setDirection(rayDir);
            ind = 3*(j+i*screenWidth);
            if (ray.raySceneIntersection(mesh, intersect) == 1) {
                rayImage[ind] = 255;//abs(alpha)*255;
                rayImage[ind+1] = intersect[1];//abs(beta)*255;
                rayImage[ind+2] = intersect[2];//255;
                //rayImage[ind] = rayImage[ind+1] = rayImage[ind+2] = 255;
            }
            else rayImage[ind] = rayImage[ind+1] = rayImage[ind+2] = 30;
        }
    }
}

void drawRays() {
    Vec3f eye = polarToCartesian(camEyePolar);
    Vec3f w = sceneCenter - eye;
    w.normalize();
    Vec3f b(0.f, 1.f, 0.f);
    Vec3f u = cross(b, w);
    u.normalize();
    Vec3f v = cross(w, u);
    float fovH = 2.f * atan(tan((fovAngle * M_PI) / 360.f) * aspectRatio);
    float alpha, beta;
    Vec3f add, rayDir;
    Ray ray(eye);
    Vec3f intersect;

    ///TODO ray trace http://www1.cs.columbia.edu/~cs4162/slides/lecture16.pdf
    setupCamera ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Erase the color and z buffers.
    glColor3f (1.f, 1.f, 1.f);
    glBegin(GL_LINES);
    for (unsigned int  j = 0; j < screenHeight; j++) {
        alpha = tan(fovH/2.f) * ((j - (screenWidth/2.f))/(screenWidth/2.f));
        for (unsigned int i = 0; i < screenWidth; i++) {
            beta = tan(fovAngle/2.f) * (((screenHeight/2.f) - i)/(screenHeight/2.f));
            add = alpha * u + beta * v - w;
            add.normalize();
            rayDir = Vec3f(1.f, 1.f, 1.f) - eye;
            ray.setDirection(rayDir);
            //if (ray.raySceneIntersection(mesh, intersect) == 1) cout << 1 << endl;
        }
    }
    glVertex3f(eye[0], eye[1], eye[2]);
    glVertex3f(rayDir[0], rayDir[1], rayDir[2]);
    glEnd();
    glBegin(GL_TRIANGLES);
    for (unsigned int i = 0; i < mesh.T.size (); i++)
        for (unsigned int j = 0; j < 3; j++) {
            const Vertex & v = mesh.V[mesh.T[i].v[j]];
            if (!mesh.materials.empty()) {
                const tinyobj::material_t& material = mesh.material(i);
                glColor3f(material.diffuse[0], material.diffuse[1], material.diffuse[2]);
            }
            glNormal3f (v.n[0], v.n[1], v.n[2]); // Specifies current normal vertex
            glVertex3f (v.p[0], v.p[1], v.p[2]); // Emit a vertex (one triangle is emitted each time 3 vertices are emitted)
        }
    glEnd ();
    glFlush(); // Ensures any previous OpenGL call has been executed
    glutSwapBuffers();
    glEnable (GL_DEPTH_TEST);
}

void display () {  
    if (rayDisplayMode)
        displayRayImage ();
        //drawRays();
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
        unsigned int numOfTriangles = mesh.T.size ();
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
    
    KDNode* node;
    
    std::vector<int> list;    
    
	for (unsigned int i=0; i < mesh.T.size(); i++) {
		list.push_back(i);
	}   	
	
	node = KDNode ::buildKDTree_test(mesh,list,0.0);
	
	/*
	Vec3f max(mesh.V[mesh.T[list[1]].v[1]].p);
	Vec3f min(mesh.V[mesh.T[list[1]].v[1]].p);	
	boundBox(max, min, mesh, list);
	Vec3f t_near;
	Vec3f t_far;
	Ray r;
	
	for (unsigned int i=0;i<3;i++){
		r.origin[i]=0.0f; r.direction[i]=i*1.0f;
	}	
    intersectBoxeRay(t_near, t_far, r);
    */
    glutMainLoop ();
    return 0;
}

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
