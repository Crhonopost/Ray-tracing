#ifndef MESH_H
#define MESH_H


#include <vector>
#include <string>
#include "Vec3.h"
#include "Ray.h"
#include "Triangle.h"
#include "Material.h"
#include "AccelerationStruct.h"

#include <GL/glut.h>

#include <cfloat>


struct BVH_Node;
struct AABB;


// -------------------------------------------
// Basic Mesh class
// -------------------------------------------

struct MeshVertex {
    inline MeshVertex () {}
    inline MeshVertex (const Vec3 & _p, const Vec3 & _n) : position (_p), normal (_n) , u(0) , v(0) {}
    inline MeshVertex (const MeshVertex & vertex) : position (vertex.position), normal (vertex.normal) , u(vertex.u) , v(vertex.v) {}
    inline virtual ~MeshVertex () {}
    inline MeshVertex & operator = (const MeshVertex & vertex) {
        position = vertex.position;
        normal = vertex.normal;
        u = vertex.u;
        v = vertex.v;
        return (*this);
    }
    // membres :
    Vec3 position; // une position
    Vec3 normal; // une normale
    float u,v; // coordonnees uv
};

struct MeshTriangle {
    inline MeshTriangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline MeshTriangle (const MeshTriangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
    }
    inline MeshTriangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;   v[1] = v1;   v[2] = v2;
    }
    unsigned int & operator [] (unsigned int iv) { return v[iv]; }
    unsigned int operator [] (unsigned int iv) const { return v[iv]; }
    inline virtual ~MeshTriangle () {}
    inline MeshTriangle & operator = (const MeshTriangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
        return (*this);
    }
    // membres :
    unsigned int v[3];
};




class Mesh {
protected:
    void build_positions_array() {
        positions_array.resize( 3 * vertices.size() );
        for( unsigned int v = 0 ; v < vertices.size() ; ++v ) {
            positions_array[3*v + 0] = vertices[v].position[0];
            positions_array[3*v + 1] = vertices[v].position[1];
            positions_array[3*v + 2] = vertices[v].position[2];
        }
    }
    void build_normals_array() {
        normalsArray.resize( 3 * vertices.size() );
        for( unsigned int v = 0 ; v < vertices.size() ; ++v ) {
            normalsArray[3*v + 0] = vertices[v].normal[0];
            normalsArray[3*v + 1] = vertices[v].normal[1];
            normalsArray[3*v + 2] = vertices[v].normal[2];
        }
    }
    void build_UVs_array() {
        uvs_array.resize( 2 * vertices.size() );
        for( unsigned int vert = 0 ; vert < vertices.size() ; ++vert ) {
            uvs_array[2*vert + 0] = vertices[vert].u;
            uvs_array[2*vert + 1] = vertices[vert].v;
        }
    }
    void build_triangles_array() {
        triangles_array.resize( 3 * triangles.size() );
        for( unsigned int t = 0 ; t < triangles.size() ; ++t ) {
            triangles_array[3*t + 0] = triangles[t].v[0];
            triangles_array[3*t + 1] = triangles[t].v[1];
            triangles_array[3*t + 2] = triangles[t].v[2];
        }
    }
public:
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;

    std::vector< float > positions_array;
    std::vector< float > normalsArray;
    std::vector< float > uvs_array;
    std::vector< unsigned int > triangles_array;

    Material material;
    void loadOFF (const std::string & filename);
    void recomputeNormals ();
    void centerAndScaleToUnit ();
    void scaleUnit ();

    AABB getBoundingBox() const;

    virtual
    void build_arrays() {
        recomputeNormals();
        build_positions_array();
        build_normals_array();
        build_UVs_array();
        build_triangles_array();
    }


    void translate( Vec3 const & translation ){
        for( unsigned int v = 0 ; v < vertices.size() ; ++v ) {
            vertices[v].position += translation;
        }
    }

    void apply_transformation_matrix( Mat3 transform ){
        for( unsigned int v = 0 ; v < vertices.size() ; ++v ) {
            vertices[v].position = transform*vertices[v].position;
        }

        //        recomputeNormals();
        //        build_positions_array();
        //        build_normals_array();
    }

    void scale( Vec3 const & scale ){
        Mat3 scale_matrix(scale[0], 0., 0.,
                0., scale[1], 0.,
                0., 0., scale[2]); //Matrice de transformation de mise à l'échelle
        apply_transformation_matrix( scale_matrix );
    }

    void rotate_x ( float angle ){
        float x_angle = angle * M_PI / 180.;
        Mat3 x_rotation(1., 0., 0.,
                        0., cos(x_angle), -sin(x_angle),
                        0., sin(x_angle), cos(x_angle));
        apply_transformation_matrix( x_rotation );
    }

    void rotate_y ( float angle ){
        float y_angle = angle * M_PI / 180.;
        Mat3 y_rotation(cos(y_angle), 0., sin(y_angle),
                        0., 1., 0.,
                        -sin(y_angle), 0., cos(y_angle));
        apply_transformation_matrix( y_rotation );
    }

    void rotate_z ( float angle ){
        float z_angle = angle * M_PI / 180.;
        Mat3 z_rotation(cos(z_angle), -sin(z_angle), 0.,
                        sin(z_angle), cos(z_angle), 0.,
                        0., 0., 1.);
        apply_transformation_matrix( z_rotation );
    }


    void draw() const;

    // RayTriangleIntersection intersect( Ray const & ray ) const {
    //     RayTriangleIntersection closestIntersection;
    //     closestIntersection.t = FLT_MAX;
    //     closestIntersection.intersectionExists = false;

    //     for(MeshTriangle meshTriangle : triangles){
    //         Vec3 p0 = vertices[meshTriangle.v[0]].position * 1.0001;
    //         Vec3 p1 = vertices[meshTriangle.v[1]].position * 1.0001;
    //         Vec3 p2 = vertices[meshTriangle.v[2]].position * 1.0001;

    //         Triangle triangle = Triangle{p0, p1, p2};

    //         RayTriangleIntersection intersection = triangle.getIntersection(ray);
    //         if(intersection.intersectionExists && intersection.t < closestIntersection.t){
    //             closestIntersection = intersection;

    //             Vec3 n0 = vertices[meshTriangle.v[0]].normal;
    //             Vec3 n1 = vertices[meshTriangle.v[1]].normal;
    //             Vec3 n2 = vertices[meshTriangle.v[2]].normal;

    //             closestIntersection.normal = n0 * intersection.w0 + n1 * intersection.w1 + n2 * intersection.w2;
    //         }
    //     }

    //     return closestIntersection;
    // }

    virtual
    RayIntersection intersect( Ray const & ray ) const{return RayIntersection();};
};


class RayTraceMesh : public Mesh{
    std::unique_ptr<BVH_Node> triangleTree;

    std::unique_ptr<AABB> boundingBox;

public:
    void build_arrays() {
        Mesh::build_arrays();
        buildTree();
    }

    AABB getBoundingBox() const;

    void buildTree(unsigned int nb_of_subdivide_tree = 3);

    void draw() const;
    RayIntersection intersect( Ray const & ray ) const;
};



#endif
