#ifndef MESH_H
#define MESH_H


#include <vector>
#include <string>
#include "Vec3.h"
#include "Ray.h"
#include "Triangle.h"
#include "Material.h"
#include "Tree.h"

#include <GL/glut.h>

#include <cfloat>


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
    KdTree<size_t> triangleTree;

    AABB boundingBox;

    std::vector< float > positions_array;
    std::vector< float > normalsArray;
    std::vector< float > uvs_array;
    std::vector< unsigned int > triangles_array;

    Material material;

    void loadOFF (const std::string & filename);
    void recomputeNormals ();
    void centerAndScaleToUnit ();
    void scaleUnit ();


    virtual
    void build_arrays() {
        recomputeNormals();
        build_positions_array();
        build_normals_array();
        build_UVs_array();
        build_triangles_array();
        buildTree();
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


    void draw() const {
        if( triangles_array.size() == 0 ) return;
        GLfloat material_color[4] = {material.diffuse_material[0],
                                     material.diffuse_material[1],
                                     material.diffuse_material[2],
                                     1.0};

        GLfloat material_specular[4] = {material.specular_material[0],
                                        material.specular_material[1],
                                        material.specular_material[2],
                                        1.0};

        GLfloat material_ambient[4] = {material.ambient_material[0],
                                       material.ambient_material[1],
                                       material.ambient_material[2],
                                       1.0};

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);

        glEnableClientState(GL_VERTEX_ARRAY) ;
        glEnableClientState (GL_NORMAL_ARRAY);
        glNormalPointer (GL_FLOAT, 3*sizeof (float), (GLvoid*)(normalsArray.data()));
        glVertexPointer (3, GL_FLOAT, 3*sizeof (float) , (GLvoid*)(positions_array.data()));
        glDrawElements(GL_TRIANGLES, triangles_array.size(), GL_UNSIGNED_INT, (GLvoid*)(triangles_array.data()));

        
        if(vertices.size() > 4){
            std::vector<AABB> aabbs;
            triangleTree.getAABBs(aabbs, boundingBox);

            for (const auto& box : aabbs) {
                drawAABB(box);
            }
        }

        // drawAABB(boundingBox);

    }


    void pushFace(Vec3 min, Vec3 max) const {
        Vec3 v1(min[0], min[1], min[2]); 
        Vec3 v2(max[0], max[1], min[2]); 
        Vec3 v3(max[0], max[1], max[2]); 
        Vec3 v4(min[0], min[1], max[2]); 

        glVertex3f(v1[0], v1[1], v1[2]);
        glVertex3f(v2[0], v2[1], v2[2]);
        glVertex3f(v3[0], v3[1], v3[2]);
        glVertex3f(v4[0], v4[1], v4[2]);
    }
    void drawAABB(const AABB& box) const{
        glLineWidth(2.f);
        glColor3f(1.0f, 0.08f, 0.58f);
        glBegin(GL_LINE_LOOP);

        Vec3 min = box.min;
        Vec3 max = box.max;

        Vec3 diag = max - min;

        pushFace(min, min + Vec3(diag[0], 0, diag[2]));
        pushFace(min, min + Vec3(0, diag[1], diag[2]));
        
        Vec3 nextPoint = min + Vec3(0, diag[1], diag[2]); 
        glVertex3f(nextPoint[0], nextPoint[1], nextPoint[2]);
        
        pushFace(max, max - Vec3(0, diag[1], diag[2]));
        pushFace(max, max - Vec3(diag[0], 0, diag[2]));
        
        nextPoint = max - Vec3(diag[0], 0, diag[2]); 
        glVertex3f(nextPoint[0], nextPoint[1], nextPoint[2]);

        glEnd();
    }

    RayTriangleIntersection intersect( Ray const & ray ) const {
        RayTriangleIntersection closestIntersection;
        closestIntersection.t = FLT_MAX;
        closestIntersection.intersectionExists = false;

        for(MeshTriangle meshTriangle : triangles){
            Vec3 p0 = vertices[meshTriangle.v[0]].position * 1.0001;
            Vec3 p1 = vertices[meshTriangle.v[1]].position * 1.0001;
            Vec3 p2 = vertices[meshTriangle.v[2]].position * 1.0001;

            Triangle triangle = Triangle{p0, p1, p2};

            RayTriangleIntersection intersection = triangle.getIntersection(ray);
            if(intersection.intersectionExists && intersection.t < closestIntersection.t){
                closestIntersection = intersection;

                Vec3 n0 = vertices[meshTriangle.v[0]].normal;
                Vec3 n1 = vertices[meshTriangle.v[1]].normal;
                Vec3 n2 = vertices[meshTriangle.v[2]].normal;

                closestIntersection.normal = n0 * intersection.w0 + n1 * intersection.w1 + n2 * intersection.w2;
            }
        }

        // Note :
        // Creer un objet Triangle pour chaque face
        // Vous constaterez des problemes de précision
        // solution : ajouter un facteur d'échelle lors de la création du Triangle : float triangleScaling = 1.000001;
        return closestIntersection;
    }

    void buildTree(unsigned int nb_of_subdivide_tree = 3){
        std::vector<Vec3> verticesPositions;
        Vec3 min, max;
        min = vertices[0].position;
        max = vertices[0].position;
        for (const auto& vertex : vertices) {
            verticesPositions.push_back(vertex.position);

            // Comparer chaque composante x, y, z du vecteur avec min et max
            for (int i = 0; i < 3; i++) { // 3 correspond à x, y, z
                if (vertex.position[i] < min[i]) {
                    min[i] = vertex.position[i];
                }
                if (vertex.position[i] > max[i]) {
                    max[i] = vertex.position[i];
                }
            }
        }

        Vec3 diag = max - min;
        diag.normalize();

        boundingBox.min = min - 0.5 * diag;
        boundingBox.max = max + 0.5 * diag;
        triangleTree = KdTree<size_t>(verticesPositions, nb_of_subdivide_tree, 0);
    }
};




#endif
