#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Vec3.h"
#include "Ray.h"
#include "Plane.h"

struct RayTriangleIntersection{
    bool intersectionExists;
    float t;
    float w0,w1,w2;
    unsigned int tIndex;
    Vec3 intersection;
    Vec3 normal;
};

class Triangle {
private:
    Vec3 m_c[3] , m_normal;
    float area;
public:
    Triangle() {}
    Triangle( Vec3 const & c0 , Vec3 const & c1 , Vec3 const & c2 ) {
        m_c[0] = c0;
        m_c[1] = c1;
        m_c[2] = c2;
        updateAreaAndNormal();
    }
    void updateAreaAndNormal() {
        Vec3 nNotNormalized = Vec3::cross( m_c[1] - m_c[0] , m_c[2] - m_c[0] );
        float norm = nNotNormalized.length();
        m_normal = nNotNormalized / norm;
        area = norm / 2.f;
    }
    void setC0( Vec3 const & c0 ) { m_c[0] = c0; } // remember to update the area and normal afterwards!
    void setC1( Vec3 const & c1 ) { m_c[1] = c1; } // remember to update the area and normal afterwards!
    void setC2( Vec3 const & c2 ) { m_c[2] = c2; } // remember to update the area and normal afterwards!
    Vec3 const & normal() const { return m_normal; }
    Vec3 projectOnSupportPlane( Vec3 const & p ) const {
        Vec3 result;
        //TODO completer
        return result;
    }
    float squareDistanceToSupportPlane( Vec3 const & p ) const {
        float result;
        //TODO completer
        return result;
    }
    float distanceToSupportPlane( Vec3 const & p ) const { return sqrt( squareDistanceToSupportPlane(p) ); }
    bool isParallelTo( Line const & L ) const {
        bool result;
        result = Vec3::dot(L.direction(), m_normal) == 0;
        return result;
    }
    Vec3 getIntersectionPointWithSupportPlane( Line const & L ) const {
        // you should check first that the line is not parallel to the plane!
        Vec3 result;

        float D = Vec3::dot(m_c[0], m_normal);
        float t = ( D - Vec3::dot(L.origin(), m_normal)) / Vec3::dot(L.direction(), m_normal);

        result = L.origin() + t * L.direction();

        return result;
    }
    void computeBarycentricCoordinates( Vec3 const & p , float & u0 , float & u1 , float & u2 ) const {
        // SOURCE: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates.html
        Vec3 b = m_c[0] - m_c[1];
        Vec3 intersectionToPoint = p - m_c[0];
        u0 = Vec3::cross(b, intersectionToPoint).norm() / 2;
        u0 /= area;
        // u0 = (intersectionToPoint - Vec3::dot(intersectionToPoint, b) * b).length();
        // u0 /= area;

        b = m_c[1] - m_c[2];
        intersectionToPoint = p - m_c[1];
        u1 = Vec3::cross(b, intersectionToPoint).norm() / 2;
        u1 /= area;
        // u1 = (intersectionToPoint - Vec3::dot(intersectionToPoint, b) * b).length();
        // u1 /= area;
        
        b = m_c[2] - m_c[0];
        intersectionToPoint = p - m_c[2];
        u2 = Vec3::cross(b, intersectionToPoint).norm() / 2;
        u2 /= area;
        // u2 = (intersectionToPoint - Vec3::dot(intersectionToPoint, b) * b).length();
        // u2 /= area;
    }

    RayTriangleIntersection getIntersection( Ray const & ray ) const {
        RayTriangleIntersection result;
        result.intersectionExists = false;
        // 1) check that the ray is not parallel to the triangle:
        if(isParallelTo(ray)) return result;

        // 2) check that the triangle is "in front of" the ray:
        Vec3 intersectionPosition = getIntersectionPointWithSupportPlane(ray);
        Vec3 collisionDirection = intersectionPosition - ray.origin();
        if(Vec3::dot(collisionDirection, ray.direction()) < 0) return result;

        // 3) check that the intersection point is inside the triangle:
        // CONVENTION: compute u,v such that p = w0*c0 + w1*c1 + w2*c2, check that 0 <= w0,w1,w2 <= 1
        float u0,u1,u2;
        computeBarycentricCoordinates(intersectionPosition, u0,u1,u2);

        float sum = u0 + u1 + u2;

        if(sum > 0 && sum <= 1){
            result.intersectionExists = true;
            result.t = (ray.origin() - intersectionPosition).length();
            result.intersection = intersectionPosition;
            result.normal = m_normal;
            result.w0 = u0;
            result.w1 = u1;
            result.w2 = u2;
        }
        

        // 4) Finally, if all conditions were met, then there is an intersection! :

        return result;
    }
};
#endif
