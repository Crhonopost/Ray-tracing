#include <AccelerationStruct.h>

#include <vector>
#include <Vec3.h>
#include <algorithm>
#include <memory>
#include <Mesh.h>
#include <map>

std::pair<bool, float> AABB::intersect(const Ray& ray, const AABB& box) {
    float tMin = 0.0f, tMax = 100.0f;

    Vec3 direction = ray.direction();
    Vec3 origin = ray.origin();
    for (int i = 0; i < 3; i++) {
        float invD = 1.0f / direction[i];
        float t0 = (box.min[i] - origin[i]) * invD;
        float t1 = (box.max[i] - origin[i]) * invD;
        if (invD < 0.0f) std::swap(t0, t1);

        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);

        if (tMin > tMax) return {false, -1.0f};
    }
    return {true, tMin};
}

BuildingTriangle::BuildingTriangle(const MeshTriangle& ref, size_t index, const std::vector<Vec3> positions){
    this->index = index;
    Vec3 a,b,c;
    a = positions[ref.v[0]];
    b = positions[ref.v[1]];
    c = positions[ref.v[2]];
    average = (a + b + c) / 3.;
    
    Vec3 min = a;
    Vec3 max = a;
    Vec3 vertices[2] = {b,c};
    for(const Vec3& pos: vertices){
        for(int i=0; i<3; i++){
            min[i] = std::min(pos[i], min[i]);
            max[i] = std::max(pos[i], max[i]);
        }
    }
    box.min = min;
    box.max = max;
}

AABB BuildingTriangle::getBoundingBox(std::vector<BuildingTriangle> bTriangles){
    AABB globalBox;
    globalBox.min = bTriangles[0].box.min;
    globalBox.max = bTriangles[0].box.max;


    for(BuildingTriangle bTriangle : bTriangles){
        Vec3 bMin = bTriangle.box.min;
        Vec3 bMax = bTriangle.box.max;
        for(int i=0; i<3; i++){
            globalBox.min[i] = std::min(globalBox.min[i], bMin[i]);
            globalBox.max[i] = std::max(globalBox.max[i], bMax[i]);
        }
    }
    globalBox.min -= Vec3(0.1);
    globalBox.max += Vec3(0.1);
    return globalBox;
}

BVH_Node BVH_Node::buildBVH(
    std::vector<Vec3> positions, 
    std::vector<MeshTriangle> triangles, 
    int deepLimit, 
    int comparisonIdx
){
    std::vector<BuildingTriangle> bTriangles;
    for(size_t i=0; i<triangles.size(); i++){
        BuildingTriangle btriangle(triangles[i], i, positions);
        bTriangles.push_back(btriangle);
    }

    Vec3 min = positions[0];
    Vec3 max = positions[0];

    for(auto pos: positions){
        for(int i=0; i<3; i++){
            min[i] = pos[i] < min[i] ? pos[i] : min[i];
            max[i] = pos[i] > max[i] ? pos[i] : max[i];
        }
    }

    AABB box;
    box.max = max;
    box.min = min;
    return BVH_Node(bTriangles, deepLimit, comparisonIdx, box);
}

BVH_Node::BVH_Node(std::vector<BuildingTriangle> bTriangles, int deepLimit, int comparisonIdx, AABB currentBox){
    boundingBox = currentBox;
    
    if(deepLimit > 0 && bTriangles.size() > 1){
        this->comparisonIdx = comparisonIdx;


        // Vec3 distance = min - max;
        // float biggest = 0;
        // for(int i=0; i<3; i++){
        //     if(abs(distance[i]) > biggest){
        //         biggest = abs(distance[i]);
        //         this->comparisonIdx = i;
        //     }
        // }

        auto sortFunction = [comparisonIdx](const BuildingTriangle& a, const BuildingTriangle& b) {
            return a.average[comparisonIdx] < b.average[comparisonIdx];
        };

        std::sort (bTriangles.begin(), bTriangles.end(), sortFunction);

        size_t mid = bTriangles.size() / 2;
        std::vector<BuildingTriangle> firstHalf(bTriangles.begin(), bTriangles.begin() + mid);
        std::vector<BuildingTriangle> secondHalf(bTriangles.begin() + mid, bTriangles.end());
        
        AABB leftBox, rightBox;
        leftBox = BuildingTriangle::getBoundingBox(firstHalf);
        rightBox = BuildingTriangle::getBoundingBox(secondHalf);

        l_child = std::make_unique<BVH_Node>(firstHalf, deepLimit - 1, (comparisonIdx + 1) % 3, leftBox);
        r_child = std::make_unique<BVH_Node>(secondHalf, deepLimit - 1, (comparisonIdx + 1) % 3, rightBox);
    } else {
        for(auto bTriangle: bTriangles){
            trianglesIdx.push_back(bTriangle.index);
        }
    }
}

void BVH_Node::getAABBs(std::vector<AABB>& aabbs) const{
    if(!canCompare()){
        aabbs.push_back(boundingBox);
        return;
    }

    if (l_child) l_child->getAABBs(aabbs);
    if (r_child) r_child->getAABBs(aabbs);
}

std::vector<size_t> BVH_Node::intersect(const Ray & ray, AABB parentBox) const{
    if(!canCompare()) {
        return this->trianglesIdx;
    }
    else {
        AABB leftBox = l_child->boundingBox;
        AABB rightBox = r_child->boundingBox;

        auto [hitL, tL] = AABB::intersect(ray, leftBox);
        auto [hitR, tR] = AABB::intersect(ray, rightBox);

        if(hitL && !hitR){
            return l_child->intersect(ray, leftBox);
        } else if (hitR && !hitL){
            return r_child->intersect(ray, rightBox);
        } else if (hitR && hitL){
            auto leftIntersection = l_child->intersect(ray, leftBox); 
            auto rightIntersection = r_child->intersect(ray, rightBox); 
            if(tL <= tR){
                leftIntersection.insert(leftIntersection.end(), rightIntersection.begin(), rightIntersection.end()); 
                return leftIntersection;
            } else {
                rightIntersection.insert(rightIntersection.end(), leftIntersection.begin(), leftIntersection.end()); 
                return rightIntersection;
            }
        }
        return std::vector<size_t>();
    }
}