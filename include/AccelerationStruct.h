#ifndef TREE_H
#define TREE_H

#include <vector>
#include <Vec3.h>
#include <algorithm>
#include <memory>
#include <Mesh.h>
#include <map>

struct AABB {
    private:
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

    public:
    Vec3 min;
    Vec3 max;

    AABB(){
        min = Vec3(INFINITY);
        max = Vec3(-INFINITY);
    }
    AABB(AABB const & box){
        min = box.min;
        max = box.max;
    }
    AABB(Vec3 min, Vec3 max) : min(min), max(max) {}

    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    static std::pair<bool, float> intersect(const Ray& ray, const AABB& box);
    Vec3 getCenter() const;

    void draw() const{
        GLfloat color[4] = {1.0f, 0.08f, 0.58f, 1.0f};

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, color);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);

        glLineWidth(2.f);
        glBegin(GL_LINE_LOOP);

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

};

struct MeshTriangle;
struct Square;
struct Sphere;
struct RayTraceMesh;

struct BuildingTriangle{
    size_t index;
    Vec3 average;
    AABB box;

    BuildingTriangle(const MeshTriangle & ref, size_t index, const std::vector<Vec3> & positions);

    static AABB getBoundingBox(const std::vector<BuildingTriangle> & bTriangles);
};


struct BuildingMesh{
    AABB boundingBox;
    void* ref;
    int type;

    BuildingMesh(void* ref, int type, const AABB &boundingBox);

    static AABB getBoundingBox(const std::vector<BuildingMesh> & bMeshes);
};
struct BVH_Node {
    bool leaf = true;
    std::unique_ptr<BVH_Node> l_child;
    std::unique_ptr<BVH_Node> r_child;

    // You can use only one of these two
    std::vector<size_t> trianglesIdx;
    std::vector<BuildingMesh> meshes;

    AABB boundingBox;

    BVH_Node() = default;

    static BVH_Node buildBVH( 
        const std::vector<Vec3> & positions, 
        const std::vector<MeshTriangle> & triangles, 
        int deepLimit, 
        int comparisonIdx
    );

    static BVH_Node buildBVH( 
        std::vector<Square> & squares,
        std::vector<Sphere> & spheres,
        std::vector<RayTraceMesh> & meshes, 
        int deepLimit = 3,
        int comparisonIdx = 0
    );

    BVH_Node(std::vector<BuildingTriangle> & bTriangles, int deepLimit, AABB currentBox);
    BVH_Node(std::vector<BuildingMesh> & bMeshes, int deepLimit, AABB currentBox, int &comparisonIdx);

    // Interdire la copie
    BVH_Node(const BVH_Node&) = delete;
    BVH_Node& operator=(const BVH_Node&) = delete;

    // Autoriser le déplacement
    BVH_Node(BVH_Node&&) = default;
    BVH_Node& operator=(BVH_Node&&) = default;

    void getAABBs(std::vector<AABB>& aabbs) const;

    std::vector<size_t> intersect(const Ray & ray, AABB parentBox) const;

    RayIntersection intersectObjects(const Ray & ray, int& totalObjects, float lightDistance = 0, bool stopAtFirst = false) const;

    void draw() const;
};

#endif
