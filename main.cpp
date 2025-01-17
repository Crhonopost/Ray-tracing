// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <atomic>

#include <algorithm>
#include "include/Vec3.h"
#include "include/Camera.h"
#include "include/Scene.h"
#include <GL/glut.h>
#include "include/ThreadPool.h"

#define STB_IMAGE_IMPLEMENTATION
#include "include/stb_image.h"

#include "include/matrixUtilities.h"

using namespace std;

#include "include/ImageLoader.h"

#include "include/Material.h"


// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 480;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static unsigned int FPS = 0;
static bool fullScreen = false;
// 70% of the threads are used for rendering
static float maxThreadPercentage = 0.6;

std::vector<Scene> scenes;
Settings settings;
unsigned int selected_scene;

std::vector< std::pair< Vec3 , Vec3 > > rays;

void printUsage () {
    cerr << endl
         << "gMini: a minimal OpenGL/GLUT application" << endl
         << "for 3D graphics." << endl
         << "Author : Tamy Boubekeur (http://www.labri.fr/~boubek)" << endl << endl
         << "Usage : ./gmini [<file.off>]" << endl
         << "Keyboard commands" << endl
         << "------------------" << endl
         << " ?: Print help" << endl
         << " w: Toggle Wireframe Mode" << endl
         << " g: Toggle Gouraud Shading Mode" << endl
         << " f: Toggle full screen mode" << endl
         << " <drag>+<left button>: rotate model" << endl
         << " <drag>+<right button>: move model" << endl
         << " <drag>+<middle button>: zoom" << endl
         << " q, <esc>: Quit" << endl << endl;
}

void usage () {
    printUsage ();
    exit (EXIT_FAILURE);
}


// ------------------------------------
void initLight () {
    GLfloat light_position[4] = {0.0, 1.5, 0.0, 1.0};
    GLfloat color[4] = { 1.0, 1.0, 1.0, 1.0};
    GLfloat ambient[4] = { 1.0, 1.0, 1.0, 1.0};

    glLightfv (GL_LIGHT1, GL_POSITION, light_position);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, color);
    glLightfv (GL_LIGHT1, GL_SPECULAR, color);
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);
}

void init () {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    initLight ();
    //glCullFace (GL_BACK);
    glDisable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);

    settings.tree_subdivide = 12;
}


// ------------------------------------
// Replace the code of this 
// functions for cleaning memory, 
// closing sockets, etc.
// ------------------------------------

void clear () {

}

// ------------------------------------
// Replace the code of this 
// functions for alternative rendering.
// ------------------------------------


void draw () {
    glEnable(GL_LIGHTING);
    scenes[selected_scene].draw();

    // draw rays : (for debug)
    //  std::cout << rays.size() << std::endl;
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glLineWidth(6);
    glColor3f(1,0,0);
    glBegin(GL_LINES);
    for( unsigned int r = 0 ; r < rays.size() ; ++r ) {
        glVertex3f( rays[r].first[0],rays[r].first[1],rays[r].first[2] );
        glVertex3f( rays[r].second[0], rays[r].second[1], rays[r].second[2] );
    }
    glEnd();
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    static unsigned int counter = 0;
    counter++;
    float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    if (currentTime - lastTime >= 1000.0f) {
        FPS = counter;
        counter = 0;
        static char winTitle [64];
        sprintf (winTitle, "Raytracer - FPS: %d", FPS);
        glutSetWindowTitle (winTitle);
        lastTime = currentTime;
    }
    glutPostRedisplay ();
}


void ray_trace_from_camera() {
    GlobalLogger::getInstance();
    const auto startTime = std::chrono::high_resolution_clock::now();

    int w = glutGet(GLUT_WINDOW_WIDTH)  ,   h = glutGet(GLUT_WINDOW_HEIGHT);
    std::cout << "Ray tracing a " << w << " x " << h << " image  (BVH: " << settings.tree_subdivide << " sections)" << std::endl;
    camera.apply();
    Vec3 pos , dir;
    //    unsigned int nsamples = 100;
    unsigned int nsamples = 30;

    std::vector<std::vector<std::pair<Vec3, Vec3>>> rays(w * h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            for (unsigned int s = 0; s < nsamples; ++s) {
                float u = ((float)(x) + (float)(rand()) / (float)(RAND_MAX)) / w;
                float v = ((float)(y) + (float)(rand()) / (float)(RAND_MAX)) / h;
                screen_space_to_world_space_ray(u, v, pos, dir);
                rays[x + y * w].emplace_back(pos, dir);
            }
        }
    }

    std::vector< Vec3 > image( w*h );
    unsigned int nbThreads = std::thread::hardware_concurrency() * maxThreadPercentage;
    ThreadPool pool(nbThreads);
    // ThreadPool pool(1);
    std::atomic<int> completedTasks(0);

    {    
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                pool.enqueue([&, x, y] {
                    ScopedLogger logger("Ray");
                    Vec3 pixel_color(0, 0, 0);
                    for (const auto& ray : rays[x + y * w]) {
                        Vec3 color = scenes[selected_scene].rayTrace(Ray(ray.first, ray.second));
                        pixel_color += color;
                    }
                    pixel_color /= nsamples;
                    image[x + y * w] = pixel_color;

                    completedTasks++;
                });
            }
        }
    }

    int totalTasks = w * h;

    while (!pool.finished()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        int completed = completedTasks.load();
        float progress = (float)completed / totalTasks * 100.0f;
        std::cout << "\rProgress: " << progress << "%" << std::flush;
    }

    std::cout << std::endl;

    
    const auto stopTime = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::seconds>(stopTime - startTime);
    cout << "Done in: " << duration.count() << "s" << endl;

    std::string filename = "./rendu.ppm";
    ofstream f(filename.c_str(), ios::binary);
    if (f.fail()) {
        cout << "Could not open file: " << filename << endl;
        return;
    }
    f << "P3" << std::endl << w << " " << h << std::endl << 255 << std::endl;
    for (int i=0; i<w*h; i++)
        f << (int)(255.f*std::min<float>(1.f,image[i][0])) << " " << (int)(255.f*std::min<float>(1.f,image[i][1])) << " " << (int)(255.f*std::min<float>(1.f,image[i][2])) << " ";
    f << std::endl;
    f.close();

    // GlobalLogger::getInstance().printAverages();
}


void key (unsigned char keyPressed, int x, int y) {
    Vec3 pos , dir;
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;
    case 'q':
    case 27:
        clear ();
        exit (0);
        break;
    case 'w':
        GLint polygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, polygonMode);
        if(polygonMode[0] != GL_FILL)
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;

    case 'r':
        camera.apply();
        rays.clear();
        ray_trace_from_camera();
        break;
    case '+':
        selected_scene++;
        if( selected_scene >= scenes.size() ) selected_scene = 0;
        scenes[selected_scene].applySettings(settings);
        break;
    case 'e':
        settings.tree_subdivide ++;
        scenes[selected_scene].applySettings(settings);
        break;
    case 'a':
        settings.tree_subdivide --;
        scenes[selected_scene].applySettings(settings);
        break;
    default:
        printUsage ();
        break;
    }
    idle ();
}

void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle ();
}

void motion (int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate (x, y);
    }
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH), (lastY-y)/static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}


void reshape(int w, int h) {
    camera.resize (w, h);
}





int main (int argc, char ** argv) {
    if (argc > 2) {
        printUsage ();
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("gMini");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);


    camera.move(0., 0., -3.1);
    selected_scene=0;
    scenes.resize(6);
    scenes[0].setup_final_scene();
    scenes[1].setup_reflexion_chamber();
    scenes[2].setup_cornell_box();
    scenes[3].setup_reflexion_scene();
    scenes[4].setup_refraction_scene();
    scenes[5].setup_test_scene();

    scenes[selected_scene].applySettings(settings);

    glutMainLoop ();
    return EXIT_SUCCESS;
}

