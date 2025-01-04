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

// struct KdTree {
//     int comparisonIdx = -1;
//     Vec3 comparedVal;

//     std::unique_ptr<KdTree> l_child;
//     std::unique_ptr<KdTree> r_child;

//     std::vector<void*> objects;

//     KdTree() = default;

//     // Old constructor
//     // KdTree(std::vector<Vec3> positions, int deepLimit, int comparisonIdx) {
//     KdTree(std::vector<std::pair<AABB, void*>> positions, int deepLimit, int comparisonIdx) {
//         if(deepLimit > 0 && positions.size() > 1){
//             this->comparisonIdx = comparisonIdx;

//             // Vec3 min = positions[0];
//             // Vec3 max = positions[0];

//             // for(auto pos: positions){
//             //     for(int i=0; i<3; i++){
//             //         min[i] = pos[i] < min[i] ? pos[i] : min[i];
//             //         max[i] = pos[i] > max[i] ? pos[i] : max[i];
//             //     }
//             // }

//             // Vec3 distance = min - max;
//             // float biggest = 0;
//             // for(int i=0; i<3; i++){
//             //     if(abs(distance[i]) > biggest){
//             //         biggest = abs(distance[i]);
//             //         this->comparisonIdx = i;
//             //     }
//             // }

//             auto sortFunction = [comparisonIdx](const Vec3& a, const Vec3& b) {
//                 return a[comparisonIdx] < b[comparisonIdx];
//             };

//             std::sort (positions.begin(), positions.end(), sortFunction);

//             size_t mid = positions.size() / 2;
//             std::vector<Vec3> firstHalf(positions.begin(), positions.begin() + mid);
//             std::vector<Vec3> secondHalf(positions.begin() + mid, positions.end());

//             this->comparedVal = Vec3(firstHalf.back());
            
//             l_child = std::make_unique<KdTree<T>>(firstHalf, deepLimit - 1, (comparisonIdx + 1) % 3);
//             r_child = std::make_unique<KdTree<T>>(secondHalf, deepLimit - 1, (comparisonIdx + 1) % 3);
//         }
//     }

//     // Interdire la copie
//     KdTree(const KdTree&) = delete;
//     KdTree& operator=(const KdTree&) = delete;

//     // Autoriser le déplacement
//     KdTree(KdTree&&) = default;
//     KdTree& operator=(KdTree&&) = default;

//     bool canCompare() const {
//         return comparisonIdx != -1;
//     }

//     KdTree<T>& getCorrespondingTree(Vec3 position){
//         if(!canCompare()) return *this;
//         if(position[comparisonIdx] <= comparedVal[comparisonIdx] && l_child){
//             return l_child->getCorrespondingTree(position);
//         } else if(position[comparisonIdx] > comparedVal[comparisonIdx] && r_child){
//             return r_child->getCorrespondingTree(position);
//         } else{
//             return *this;
//         }
//     }


//     void getAABBs(std::vector<AABB>& aabbs, const AABB& currentBox) const{
//         aabbs.push_back(currentBox);

//         if(!canCompare()) return;

//         AABB leftBox = currentBox;
//         AABB rightBox = currentBox;
//         leftBox.max[comparisonIdx] = comparedVal[comparisonIdx];
//         rightBox.min[comparisonIdx] = comparedVal[comparisonIdx];

//         if (l_child) l_child->getAABBs(aabbs, leftBox);
//         if (r_child) r_child->getAABBs(aabbs, rightBox);
//     }

//     std::vector<T> intersect(const Ray & ray, AABB parentBox) const{
//         if(!canCompare()) {
//             return this->values;
//         }
//         else {
//             AABB leftBox = parentBox;
//             AABB rightBox = parentBox;
//             leftBox.max[comparisonIdx] = comparedVal[comparisonIdx];
//             rightBox.min[comparisonIdx] = comparedVal[comparisonIdx];

//             auto [hitL, tL] = AABB::intersect(ray, leftBox);
//             auto [hitR, tR] = AABB::intersect(ray, rightBox);

//             if(hitL && !hitR){
//                 return l_child->intersect(ray, leftBox);
//             } else if (hitR && !hitL){
//                 return r_child->intersect(ray, rightBox);
//             } else if (hitR && hitL){
//                 auto leftIntersection = l_child->intersect(ray, leftBox); 
//                 auto rightIntersection = r_child->intersect(ray, rightBox); 
//                 if(tL <= tR){
//                     leftIntersection.insert(leftIntersection.end(), rightIntersection.begin(), rightIntersection.end()); 
//                     return leftIntersection;
//                 } else {
//                     rightIntersection.insert(rightIntersection.end(), leftIntersection.begin(), leftIntersection.end()); 
//                     return rightIntersection;
//                 }
//             }
//             return std::vector<T>();
//         }
//     }
// };
