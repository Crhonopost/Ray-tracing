#ifndef MATERIAL_H
#define MATERIAL_H

#include "imageLoader.h"
#include "Vec3.h"
#include <cmath>

#include <GL/glut.h>

enum MaterialType {
    Material_Diffuse_Blinn_Phong ,
    Material_Glass,
    Material_Mirror
};


struct Material {
    Vec3 ambient_material;
    Vec3 diffuse_material;
    Vec3 specular_material;
    double shininess;

    float index_medium;
    float transparency;

    MaterialType type;

    Material() {
        type = Material_Diffuse_Blinn_Phong;
        transparency = 0.0;
        index_medium = 1.0;
        ambient_material = Vec3(0., 0., 0.);
    }

    static Vec3 reflect(Vec3 direction, Vec3 normal){
        Vec3 parallel = Vec3::dot(-1. * direction, normal) * normal - (-1. * direction);
        Vec3 outDirection = normal + parallel;
        outDirection.normalize();
        return outDirection;
    }

    static Vec3 refract(Vec3 direction, Vec3 normal, float ri, float cosTheta){
        Vec3 rOutPerp = ri * (direction + cosTheta * normal);
        Vec3 rOutPara = -std::sqrt(std::fabs(1.0 - rOutPerp.squareLength())) * normal;
        Vec3 outDirection = rOutPara + rOutPerp;
        outDirection.normalize();
        return outDirection;
    }
    
    static double reflectance(double cosine, double refraction_index) {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0*r0;
        return r0 + (1-r0)*std::pow((1 - cosine),5);
    }
};



#endif // MATERIAL_H
