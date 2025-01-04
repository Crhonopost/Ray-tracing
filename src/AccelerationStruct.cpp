#include <AccelerationStruct.h>

#include <vector>
#include <Vec3.h>
#include <algorithm>
#include <memory>
#include <Mesh.h>
#include <map>



AABB::AABB(Vec3 center, float radius){
    Vec3 diag(radius * 0.34);
    min = center - diag;
    max = center + diag;
}
AABB::AABB(Vec3 bottomLeft, Vec3 topLeft, Vec3 bottomRight){
    Vec3 vertices[3] = {bottomLeft, topLeft, bottomRight};
    min = bottomLeft;
    max = bottomLeft;
    for(const Vec3& pos: vertices){
        for(int i=0; i<3; i++){
            min[i] = std::min(pos[i], min[i]);
            max[i] = std::max(pos[i], max[i]);
        }
    }
}

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

BuildingTriangle::BuildingTriangle(const MeshTriangle & ref, size_t index, const std::vector<Vec3> & positions){
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

AABB BuildingTriangle::getBoundingBox(const std::vector<BuildingTriangle> & bTriangles){
    AABB globalBox(bTriangles[0].box);

    for(BuildingTriangle bTriangle : bTriangles){
        Vec3 bMin = bTriangle.box.min;
        Vec3 bMax = bTriangle.box.max;
        for(int i=0; i<3; i++){
            globalBox.min[i] = std::min(globalBox.min[i], bMin[i]);
            globalBox.max[i] = std::max(globalBox.max[i], bMax[i]);
        }
    }
    return globalBox;
}

BVH_Node BVH_Node::buildBVH(
    const std::vector<Vec3> & positions, 
    const std::vector<MeshTriangle> & triangles, 
    int deepLimit, 
    int comparisonIdx
){
    std::vector<BuildingTriangle> bTriangles;
    for(size_t i=0; i<triangles.size(); i++){
        BuildingTriangle btriangle(triangles[i], i, positions);
        bTriangles.push_back(btriangle);
    }

    AABB box;

    for(auto pos: positions){
        for(int i=0; i<3; i++){
            box.min[i] = pos[i] < box.min[i] ? pos[i] : box.min[i];
            box.max[i] = pos[i] > box.max[i] ? pos[i] : box.max[i];
        }
    }

    return BVH_Node(bTriangles, deepLimit, box);
}

BVH_Node::BVH_Node(std::vector<BuildingTriangle> & bTriangles, int deepLimit, AABB currentBox){
    boundingBox = currentBox;
    
    if(deepLimit > 0 && bTriangles.size() > 1){
        leaf = false;

        Vec3 diag = boundingBox.max - boundingBox.min;

        // Trouve l'index de l'axe le plus long de la boite
        int comparisonIdx = diag[0] > std::max(diag[1], diag[2]) ? 0 : diag[1] > diag[2] ? 1 : 2;

        Vec3 center = boundingBox.min + diag / 2.;

        auto middle = std::partition(bTriangles.begin(), bTriangles.end(),
        [center, comparisonIdx](const BuildingTriangle& bTriangle) {
            return bTriangle.average[comparisonIdx] < center[comparisonIdx];
        });

        std::vector<BuildingTriangle> firstHalf(bTriangles.begin(), middle);
        std::vector<BuildingTriangle> secondHalf(middle, bTriangles.end());

        
        AABB emptyBox;
        std::vector<BuildingTriangle> emptyVec;
        
        if(firstHalf.size() >= 1){
            AABB firstBox = BuildingTriangle::getBoundingBox(firstHalf);
            l_child = std::make_unique<BVH_Node>(firstHalf, deepLimit - 1, firstBox);
        } else {
            l_child = std::make_unique<BVH_Node>(emptyVec, 0, emptyBox);
        }
        if(secondHalf.size() >= 1){
            AABB secondBox = BuildingTriangle::getBoundingBox(secondHalf);
            r_child = std::make_unique<BVH_Node>(secondHalf, deepLimit - 1, secondBox);
        } else {
            r_child = std::make_unique<BVH_Node>(emptyVec, 0, emptyBox);
        }


    } else {
        for(const auto & bTriangle: bTriangles){
            trianglesIdx.push_back(bTriangle.index);
        }
    }
}

void BVH_Node::getAABBs(std::vector<AABB>& aabbs) const{
    if(leaf){
        aabbs.push_back(boundingBox);
        return;
    }

    if (l_child) l_child->getAABBs(aabbs);
    if (r_child) r_child->getAABBs(aabbs);
}

std::vector<size_t> BVH_Node::intersect(const Ray & ray, AABB parentBox) const{
    if(leaf) {
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