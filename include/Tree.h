#ifndef TREE_H
#define TREE_H

#include <vector>
#include <Vec3.h>
#include <algorithm>
#include <memory>

struct AABB {
    Vec3 min;
    Vec3 max;
};


template <typename T>
struct KdTree {
    int comparisonIdx = -1;
    Vec3 comparedVal;

    std::unique_ptr<KdTree<T>> l_child;
    std::unique_ptr<KdTree<T>> r_child;

    std::vector<T> values;

    KdTree<T>() = default;

    KdTree<T>(std::vector<Vec3> positions, int deepLimit, int comparisonIdx) {
        if(deepLimit > 0 && positions.size() > 1){
            this->comparisonIdx = comparisonIdx;

            auto sortFunction = [comparisonIdx](const Vec3& a, const Vec3& b) {
                return a[comparisonIdx] < b[comparisonIdx];
            };

            std::sort (positions.begin(), positions.end(), sortFunction);

            size_t mid = positions.size() / 2;
            std::vector<Vec3> firstHalf(positions.begin(), positions.begin() + mid);
            std::vector<Vec3> secondHalf(positions.begin() + mid, positions.end());

            this->comparedVal = Vec3(firstHalf.back());
            
            l_child = std::make_unique<KdTree<T>>(firstHalf, deepLimit - 1, (comparisonIdx + 1) % 3);
            r_child = std::make_unique<KdTree<T>>(secondHalf, deepLimit - 1, (comparisonIdx + 1) % 3);
        }
    }

    // Interdire la copie
    KdTree(const KdTree&) = delete;
    KdTree& operator=(const KdTree&) = delete;

    // Autoriser le déplacement
    KdTree(KdTree&&) = default;
    KdTree& operator=(KdTree&&) = default;

    bool canCompare() const {
        return comparisonIdx != -1;
    }

    KdTree<T> getCorrespondingTree(Vec3 position) const{
        if(position[comparisonIdx] <= comparedVal[comparisonIdx] && l_child){
            return l_child->getCorrespondingTree(position);
        } else if(position[comparisonIdx] > comparedVal[comparisonIdx] && r_child){
            return r_child->getCorrespondingTree(position);
        } else{
            return this;
        }
    }


    void getAABBs(std::vector<AABB>& aabbs, const AABB& currentBox) const{
        aabbs.push_back(currentBox);

        if(!canCompare()) return;

        AABB leftBox = currentBox;
        AABB rightBox = currentBox;
        leftBox.max[comparisonIdx] = comparedVal[comparisonIdx];
        rightBox.min[comparisonIdx] = comparedVal[comparisonIdx];

        if (l_child) l_child->getAABBs(aabbs, leftBox);
        if (r_child) r_child->getAABBs(aabbs, rightBox);
    }
};


#endif