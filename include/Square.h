#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>
#include "AccelerationStruct.h"

struct RaySquareIntersection{
    bool intersectionExists;
    float t;
    float u,v;
    Vec3 intersection;
    Vec3 normal;
};


class Square : public Mesh {
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
           float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) : Mesh() {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad( Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
                  float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector , upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector*width;
        m_up_vector = m_up_vector*height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;                                      vertices[0].u = uMin; vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;                     vertices[1].u = uMax; vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;       vertices[2].u = uMax; vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;                        vertices[3].u = uMin; vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;
    }

    RayIntersection intersect(const Ray &ray) const {
        RaySquareIntersection intersection;
        intersection.intersectionExists = false;

        // solution provisoire trouvée par Tom pour récupérer les bonnes valeurs du quad
        Vec3 m_bottom_left = vertices[0].position;
        Vec3 m_right_vector = vertices[1].position - vertices[0].position;
        Vec3 m_up_vector = vertices[3].position - vertices[0].position;
        Vec3 m_normal = Vec3::cross(m_right_vector, m_up_vector);
        m_normal.normalize();

        // Face culling
        if(Vec3::dot(ray.direction(), m_normal) > 0) return RayIntersection();

        // Intersection du rayon avec le plan
        float D = Vec3::dot(m_bottom_left, m_normal);
        float t = ( D - Vec3::dot(ray.origin(), m_normal)) / Vec3::dot(ray.direction(), m_normal);

        // Il y a une intersection avec le plan
        if(t>0 && !std::isinf(t)){
            Vec3 position = ray.origin() + t * ray.direction();

            Vec3 d = position - m_bottom_left;
            float u = Vec3::dot(d, m_right_vector) / m_right_vector.squareLength();
            float v = Vec3::dot(d, m_up_vector) / m_up_vector.squareLength();

            // Le point se situe dans les limites du quad
            if(u>=0 && u<=1.0 && v>=0 && v<=1.0){
                intersection.intersectionExists = true;
                intersection.t = t;
                intersection.intersection = position;
                intersection.normal = m_normal;
                intersection.u = u;
                intersection.v = v;
            }

        }

        RayIntersection result;
        result.intersectionExists = intersection.intersectionExists;
        result.t = intersection.t;
        result.intersection = intersection.intersection;
        result.normal = intersection.normal;
        result.position = ray.origin() + ray.direction() * intersection.t;
        result.u = intersection.u;
        result.v = intersection.v;
        result.material = material;

        return result;
    }

    AABB getBoundingBox() const{
        Vec3 min = vertices[0].position;
        Vec3 max = vertices[0].position;
        for(const auto & vertex : vertices){
            for(int i=0; i<3; i++){
                min[i] = std::min(min[i], vertex.position[i]);
                max[i] = std::max(max[i], vertex.position[i]);
            }
        }
        return AABB(min, max);
    }
    
};
#endif // SQUARE_H
