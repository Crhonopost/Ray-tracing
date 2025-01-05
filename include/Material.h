#ifndef MATERIAL_H
#define MATERIAL_H

#include "ImageLoader.h"
#include "Vec3.h"
#include <cmath>
#include <GL/glut.h>

enum MaterialType {
    Material_Diffuse_Blinn_Phong ,
    Material_Glass,
    Material_Mirror,
    Material_Checker
};


struct Material {
    Vec3 ambient_material;
    Vec3 diffuse_material;
    Vec3 specular_material;
    double shininess;

    ppmLoader::ImageRGB texture;
    ppmLoader::ImageRGB normalMap;

    bool hasTexture = false;
    bool hasNormalMap = false;
    double materialScale = 1.;

    float index_medium;
    float transparency;

    float reflectivity = 0.0f;

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
        float k = 1.0 - rOutPerp.squareLength();
        if (k < 0.0) return Vec3(0.0, 0.0, 0.0); // Pas de réfraction possible
        Vec3 rOutPara = -std::sqrt(k) * normal;
        // Vec3 rOutPara = -std::sqrt(std::fabs(1.0 - rOutPerp.squareLength())) * normal;
        Vec3 outDirection = rOutPara + rOutPerp;
        outDirection.normalize();
        return outDirection;
    }
    
    // TODO: source ?
    static double reflectance(double cosine, double refraction_index) {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0*r0;
        return r0 + (1-r0)*std::pow((1 - cosine),5);
    }

    Vec3 getPixelAt(double u, double v, ppmLoader::ImageRGB & texture){
        int pixelU = (int) (u * (double) texture.w * materialScale);
        int pixelV = (int) (v * (double) texture.h * materialScale);

        pixelU = pixelU % texture.w;
        pixelV = pixelV % texture.h;

        ppmLoader::RGB rgb = texture.data[pixelV * texture.w + pixelU];

        return Vec3(rgb.r, rgb.g, rgb.b) / 255.;
    }

    Vec3 getColorAt(double u, double v){
        return getPixelAt(u, v, texture);
    }

    Vec3 applyTBN(const Vec3 & normal, double u, double v){
        Vec3 normalFromMap = getNormalAt(u, v);
        Vec3 tangent = Vec3(1., 0., 0.);
        Vec3 bitangent = Vec3::cross(normal, tangent);
        Mat3 TBN = Mat3(tangent[0], bitangent[0], normal[0],
                        tangent[1], bitangent[1], normal[1],
                        tangent[2], bitangent[2], normal[2]);
        return TBN * normalFromMap;
    }
private:
    Vec3 getNormalAt(double u, double v){
        Vec3 normal = getPixelAt(u, v, normalMap);
        normal = 2. * normal - Vec3(1.);
        normal.normalize();
        return normal;
    }
};

// https://raytracing.github.io/books/RayTracingTheNextWeek.html#texturemapping/constantcolortexture
struct CheckerTexture{
    Vec3 colorA, colorB;
    float inv_scale;

    CheckerTexture(float scale, Vec3 colorA, Vec3 colorB){
        this->colorA = colorA;
        this->colorB = colorB;

        inv_scale = 1. / scale;
    }

    Vec3 getPixelAt(double u, double v, const Vec3 & p){
        auto xInteger = int(std::floor(inv_scale * p[0]));
        auto yInteger = int(std::floor(inv_scale * p[1]));
        auto zInteger = int(std::floor(inv_scale * p[2]));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? colorA : colorB;
    }
};



#endif // MATERIAL_H
