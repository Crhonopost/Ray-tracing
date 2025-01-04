#include <float.h>
#include "Material.h"

#ifndef RAY_H
#define RAY_H
#include "Line.h"
class Ray : public Line {
public:
    Ray() : Line() {}
    Ray( Vec3 const & o , Vec3 const & d ) : Line(o,d) {}
};

class RayIntersection {
public:
    bool intersectionExists;
    float t;
    Vec3 intersection;
    Vec3 normal;
    Vec3 position;
    double u, v;
    Material material;
    RayIntersection() 
        : intersectionExists(false), material(Material()), t(FLT_MAX) {}; 
};

// struct RaySceneIntersection{
//     bool intersectionExists;
//     unsigned int typeOfIntersectedObject;
//     unsigned int objectIndex;
//     float t;
//     RayTriangleIntersection rayMeshIntersection;
//     RaySphereIntersection raySphereIntersection;
//     RaySquareIntersection raySquareIntersection;
//     RaySceneIntersection() : intersectionExists(false) , t(FLT_MAX) {}
// };
#endif
