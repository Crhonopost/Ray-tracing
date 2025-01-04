#ifndef TREE_H
#define TREE_H

#include <vector>
#include <Vec3.h>
#include <algorithm>
#include <memory>
#include <Mesh.h>
#include <map>

struct AABB {
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
    AABB(Vec3 center, float radius);
    AABB(Vec3 bottomLeft, Vec3 topLeft, Vec3 bottomRight);

    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    static std::pair<bool, float> intersect(const Ray& ray, const AABB& box);
    Vec3 getCenter() const;
};

struct MeshTriangle;

struct BuildingTriangle{
    size_t index;
    Vec3 average;
    AABB box;

    BuildingTriangle(const MeshTriangle & ref, size_t index, const std::vector<Vec3> & positions);

    static AABB getBoundingBox(const std::vector<BuildingTriangle> & bTriangles);
};

struct BVH_Node {
    bool leaf = true;
    std::unique_ptr<BVH_Node> l_child;
    std::unique_ptr<BVH_Node> r_child;

    std::vector<size_t> trianglesIdx;

    AABB boundingBox;

    BVH_Node() = default;

    static BVH_Node buildBVH( 
        const std::vector<Vec3> & positions, 
        const std::vector<MeshTriangle> & triangles, 
        int deepLimit, 
        int comparisonIdx
    );

    BVH_Node(std::vector<BuildingTriangle> & bTriangles, int deepLimit, AABB currentBox);

    // Interdire la copie
    BVH_Node(const BVH_Node&) = delete;
    BVH_Node& operator=(const BVH_Node&) = delete;

    // Autoriser le déplacement
    BVH_Node(BVH_Node&&) = default;
    BVH_Node& operator=(BVH_Node&&) = default;

    void getAABBs(std::vector<AABB>& aabbs) const;

    std::vector<size_t> intersect(const Ray & ray, AABB parentBox) const;
};

#endif
