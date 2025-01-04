#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"


#include <GL/glut.h>


enum LightType {
    LightType_Spherical,
    LightType_Quad
};


struct Light {
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Square quad;

    float powerCorrection;

    Light() : powerCorrection(1.0) {}

};


struct Settings{
    unsigned int tree_subdivide;
};


class Scene {
    std::vector< RayTraceMesh > meshes;
    std::vector< Sphere > spheres;
    std::vector< Square > squares;

    std::vector< Light > lights;

    CheckerTexture checkerTexture = CheckerTexture(4, Vec3(0), Vec3(1));

public:
    void applySettings(Settings set){
        for(auto& mesh : meshes){
            mesh.buildTree(set.tree_subdivide);
        }
    }


    Scene() {
    }

    void draw() {
        // iterer sur l'ensemble des objets, et faire leur rendu :
        for(auto & mesh : meshes){
            mesh.draw();
        }
        for(auto & sphere : spheres){
            sphere.draw();
        }
        for(auto & square : squares){
            square.draw();
        }
    }




    RayIntersection computeIntersection(Ray const & ray) {
        RayIntersection result;

        for(auto & mesh : meshes){
            RayIntersection intersection = mesh.intersect(ray);
            if(intersection.intersectionExists && result.t > intersection.t){
                result = intersection;
            }
        }
        for(auto & sphere : spheres){
            RayIntersection intersection = sphere.intersect(ray);
            if(intersection.intersectionExists && result.t > intersection.t){
                result = intersection;
            }
        }
        for(auto & square : squares){
            RayIntersection intersection = square.intersect(ray);
            if(intersection.intersectionExists && result.t > intersection.t){
                result = intersection;
            }
        }

        return result;
    }

    Vec3 phong(Vec3 sourceIntensity, Vec3 lightPos, Vec3 intersectionPos, Vec3 surfaceNormal, Vec3 directionToEye, Material material){
        Vec3 directionToLight = lightPos - intersectionPos;
        directionToLight.normalize();
        
        float theta = Vec3::dot(surfaceNormal, directionToLight);

        // ambiant
        Vec3 ambiant = Vec3::compProduct(sourceIntensity, material.ambient_material);

        // diffuse
        Vec3 diffuse = Vec3::compProduct(sourceIntensity, material.diffuse_material) * std::max(theta, 0.f);

        // specular
        Vec3 r = 2. * theta * surfaceNormal - directionToLight;
        float rDotV = std::max(0.0f,Vec3::dot(r, directionToEye));
        Vec3 specular = Vec3::compProduct(sourceIntensity, material.specular_material) * pow(rDotV, material.shininess);

        return ambiant + diffuse + specular;
    }

    float sampleSphereLight(Vec3 intersectionPos, Vec3 lightPos, float radius, int sampleCount){
        float shadowCount = 0.;
        float degToRad = M_PI / 180.;
        for(int i=0; i<sampleCount; i++){
            // angles de rotations
            float randY =  (random() % 360) * degToRad;
            float randX =  (random() % 360) * degToRad;

            // rotation du vec (0,0,1) autour de l'axe Y
            Vec3 position = {0, -sin(randY),cos(randY)};

            position[0] = -sin(randX) * position[2];
            position[2] = cos(randX) * position[2];

            position *= radius;

            Vec3 areaLightPos = lightPos + position;
            // Il y a une ombre sur le chemin des coordonnées aléatoires vers le point
            if(isInShadow(intersectionPos, areaLightPos)){
                shadowCount += 1.;
            }
        }

        return shadowCount / (float) sampleCount;
    }

    // float sampleSquareLight(Vec3 intersectionPos, Vec3 lightPos, float areaSize, int sampleCount){
    //     int halfRand = RAND_MAX/2;
    //     Vec3 lightDirection = lightPos - intersectionPos;
    //     Vec3 up = Vec3(0, 1, 0);
    //     Vec3 h = Vec3::cross(up, lightDirection);
    //     h.normalize();
    //     Vec3 v = Vec3::cross(up, h);

    //     float shadowCount = 0.;

    //     for(int i=0; i<sampleCount; i++){
    //         float randX = areaSize / (float) (rand() - halfRand);
    //         float randY = areaSize / (float) (rand() - halfRand);

    //         Vec3 areaLightPos = lightPos + randX * h + randY * v;
    //         if(isInShadow(intersectionPos, areaLightPos)){
    //             shadowCount += 1.;
    //         }
    //     }

    //     return shadowCount / (float) sampleCount;
    // }


    bool isInShadow(Vec3 intersectionPos, Vec3 lightPos){
        Vec3 direction = lightPos - intersectionPos;
        float length = direction.length();
        direction.normalize();
        Ray ray = Ray(intersectionPos, direction);

        for(auto & mesh : meshes){
            RayIntersection intersection = mesh.intersect(ray);
            if(intersection.intersectionExists && intersection.t > 0.001 && intersection.t < length){
                return true;
            }
        }
        for(auto & sphere : spheres){
            RayIntersection intersection = sphere.intersect(ray);
            if(intersection.intersectionExists && intersection.t > 0.001 && intersection.t < length){
                return true;
            }
        }
        for(auto & square : squares){
            RayIntersection intersection = square.intersect(ray);
            if(intersection.intersectionExists && intersection.t > 0.001 && intersection.t < length){
                return true;
            }
        }

        return false;
    }





    Vec3 rayTraceRecursive( Ray ray , int NRemainingBounces ) {
        RayIntersection raySceneIntersection = computeIntersection(ray);
        Vec3 color = Vec3{0.1, 0.2, 0.3};
        Light &light = lights[0];

        if(raySceneIntersection.intersectionExists){
            Vec3 intersectionPosition, intersectionNormal, intersectionUV;
            Material material;

            intersectionPosition = raySceneIntersection.position;
            intersectionNormal = raySceneIntersection.normal;
            intersectionUV = Vec3(raySceneIntersection.u, raySceneIntersection.v, 0);
            material = raySceneIntersection.material;

            if(material.type == MATERIAL_TEXTURE){
                color = material.getPixelAt(intersectionUV[0], intersectionUV[1]);;
                material.ambient_material = color;
                material.diffuse_material = color;
                material.specular_material = color;
            }

            if(material.type == Material_Diffuse_Blinn_Phong || NRemainingBounces<=0){
                
                color = phong(light.material, light.pos, intersectionPosition, intersectionNormal, -1 * ray.direction(), material);

            } else if(material.type == Material_Mirror) {
                Vec3 newDirection = Material::reflect(ray.direction(), intersectionNormal);
                Ray newRay = Ray(intersectionPosition + intersectionNormal*0.001, newDirection);
                color = Vec3::compProduct(material.specular_material, rayTraceRecursive(newRay, NRemainingBounces-1));
            } else if(material.type == MATERIAL_TEXTURE){
                color = material.getPixelAt(intersectionUV[0], intersectionUV[1]);
            } else if(material.type == Material_Checker){
                color = checkerTexture.getPixelAt(intersectionUV[0], intersectionUV[1], intersectionPosition);
            } else if(material.type == Material_Glass) {
                float cosTheta = std::fmin(Vec3::dot(-1. * ray.direction(), intersectionNormal), 1.);
                float sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);
                bool isInside = cosTheta < 0.;

                float refractionIndex = isInside ? material.index_medium : 1. / material.index_medium;
                double reflectance = Material::reflectance(cosTheta, refractionIndex); 
                
                Ray newRay;
                if(refractionIndex * sinTheta > 1.0){// || reflectance > std::rand() / (RAND_MAX + 1.0)){
                    Vec3 newDirection = Material::reflect(ray.direction(), intersectionNormal);
                    newRay = Ray(intersectionPosition + intersectionNormal*0.001, newDirection);
                }else{
                    double offsetDir = isInside ? 0.00001 : -0.00001;
                    Vec3 newDirection = Material::refract(ray.direction(), intersectionNormal, refractionIndex, cosTheta);
                    newRay = Ray(intersectionPosition + (offsetDir * intersectionNormal), newDirection);            
                }
                
                color = rayTraceRecursive(newRay, NRemainingBounces-1);
            }
        // shadow casting
        // float shadowStrength = sampleSphereLight(intersectionPosition, light.pos, light.radius, 45);
        float shadowStrength = isInShadow(intersectionPosition, light.pos);
        // float shadowStrength = 0.;
        color *= (1. - (shadowStrength * shadowStrength));

        }
        return color;
    }


    Vec3 rayTrace( Ray const & rayStart ) {
        Vec3 color = rayTraceRecursive(rayStart, 4);
        return color;
    }

    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,1.,1 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
    }

    void setup_single_square() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.8,0.8,0.8 );
            s.material.specular_material = Vec3( 0.8,0.8,0.8 );
            s.material.shininess = 20;
        }
    }

    void setup_cornell_box(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 1.5, 0.0 );
            light.radius = .5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        { //Back Wall
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.5,0.,0.5 );
            s.material.specular_material = Vec3( 0.5,0.,0.5 );
            s.material.shininess = 16;
            s.material.type = MATERIAL_TEXTURE;
            ppmLoader::load_ppm(s.material.texture, "assets/img/sphereTextures/s4.ppm");
        }

        { //Left Wall
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
        }

        { //Right Wall
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.,1.,0. );
            s.material.specular_material = Vec3( 0.,1.,0. );
            s.material.shininess = 16;
        }

        { //Floor
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }

        { //Ceiling
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }

        { //Front Wall
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(180);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }


        { //GLASS Sphere
            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.0, -1.25, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 2.9;
        }


        { //MIRRORED Sphere
            spheres.resize(spheres.size() + 1); 
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.0, -1.25, -0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 0.;
            s.material.index_medium = 0.;
        }

        { // Moon
            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-0.75, 0, -0.75);
            s.m_radius = 0.5f;
            s.build_arrays();
            s.material.type = MATERIAL_TEXTURE;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            ppmLoader::load_ppm(s.material.texture, "assets/img/sphereTextures/s1.ppm");
        }

        // {
        //     meshes.resize( meshes.size() + 1 );
        //     Mesh & mesh = meshes[meshes.size() - 1];
        //     mesh.loadOFF("assets/models/gorilla18_fixed.obj.off");
        //     mesh.scale(Vec3(0.0125));
        //     mesh.translate(Vec3{0, -1.05, 0});
        //     mesh.build_arrays();
        //     mesh.material.type = Material_Glass;
        //     mesh.material.diffuse_material = Vec3( 1.,1.,1. );
        //     mesh.material.specular_material = Vec3( 1.,1.,1. );
        //     mesh.material.shininess = 16;
        //     mesh.material.transparency = 1.0;
        //     mesh.material.index_medium = 1.4;
        // }

    }

    void setup_refraction_scene(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 3.5, 0.0 );
            light.radius = .5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        { //bottom left
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(-1., -1., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1, 0, 0 );
            s.material.specular_material = Vec3( 1, 0, 0 );
            s.material.shininess = 16;
        }
        { //bottom right
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(1., -1., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0, 1, 0 );
            s.material.specular_material = Vec3( 0, 1, 0 );
            s.material.shininess = 16;
        }
        { //top left
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(-1., 1, -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0, 0, 1 );
            s.material.specular_material = Vec3( 0, 0, 1 );
            s.material.shininess = 16;
        }
        { //top right
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(1, 1, -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1, 1, 1 );
            s.material.specular_material = Vec3( 1, 1, 1 );
            s.material.shininess = 16;
        }
        { //Ceiling
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(80., 80., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.75,1.,0.75 );
            s.material.specular_material = Vec3( 0.75,1.,0.75 );
            s.material.shininess = 16;

            s.material.type = Material_Checker;
        }
        { //GLASS Sphere
            spheres.resize(spheres.size() + 1);
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0, 0, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
        }
    }

    void setup_test_scene(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 1.0, 2.5, 0.0 );
            light.radius = .5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        { //Floor
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }

        { //Back Wall
            squares.resize(squares.size() + 1);
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.5,0.,0.5 );
            s.material.specular_material = Vec3( 0.5,0.,0.5 );
            s.material.shininess = 16;
        }

        // {
        //     meshes.resize( meshes.size() + 1 );
        //     Mesh & mesh = meshes[meshes.size() - 1];
        //     // mesh.rotate_x(-90);
        //     mesh.translate(Vec3{0, -1.05, 0});
        //     mesh.build_arrays();
        //     mesh.material.diffuse_material = Vec3( 1.,1.,1. );
        //     mesh.material.specular_material = Vec3( 1.,1.,1. );
        //     mesh.material.shininess = 16;
        // }
        {
            meshes.resize( meshes.size() + 1 );
            Mesh & mesh = static_cast<Mesh&>(meshes[meshes.size() - 1]);
            mesh.loadOFF("assets/models/gorilla18_fixed.obj.off");
            mesh.scale(Vec3(0.0125));
            mesh.translate(Vec3{0, -1.05, 0});
            mesh.build_arrays();
            // mesh.material.type = Material_Glass;
            mesh.material.diffuse_material = Vec3( 1.,1.,1. );
            mesh.material.specular_material = Vec3( 1.,1.,1. );
            mesh.material.shininess = 16;
            // mesh.material.transparency = 1.0;
            // mesh.material.index_medium = 1.4;
        }

        // {
        //     meshes.resize( meshes.size() + 1 );
        //     Mesh & mesh = meshes[meshes.size() - 1];
        //     mesh.loadOFF("assets/models/dragon.off");
        //     mesh.scale(Vec3(10));
        //     mesh.translate(Vec3{0, -1.05, 0});
        //     mesh.build_arrays();
        //     mesh.material.type = Material_Glass;
        //     mesh.material.diffuse_material = Vec3( 1.,0.,0. );
        //     mesh.material.specular_material = Vec3( 1.,0.,0. );
        //     mesh.material.shininess = 16;
        //     mesh.material.transparency = 1.0;
        //     mesh.material.index_medium = 1.4;
        // }

        
    //     {
    //         meshes.resize( meshes.size() + 1 );
    //         Mesh & mesh = meshes[meshes.size() - 1];
    //         mesh.loadOFF("assets/models/blob.off");
    //         mesh.rotate_x(180);
    //         mesh.rotate_y(45);
    //         mesh.translate(Vec3{-0.125, -1.05, -1.25});
    //         mesh.build_arrays();
    //         // mesh.material.type = Material_Glass;
    //         mesh.material.diffuse_material = Vec3( 1.,1.,1. );
    //         mesh.material.specular_material = Vec3( 1.,1.,1. );
    //         mesh.material.shininess = 16;
    //         mesh.material.transparency = 0.5;
    //         mesh.material.index_medium = 0.;
    //     }
    }
};


#endif
