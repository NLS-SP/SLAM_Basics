//
// Created by Robotics_qi on 2020/8/21.
//

#ifndef BASIC_ICP_QUICK_OCTREE_H
#define BASIC_ICP_QUICK_OCTREE_H

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace Utils{
    template<typename PointT, int D>
    struct access{};

    template<class PointT>
    struct access<PointT, 0>{
        static float get(const PointT& p){
            return p.x;
        }
    };

    template<class PointT>
    struct access<PointT, 1>{
        static float get(const PointT& p){
            return p.y;
        }
    };

    template<class PointT>
    struct access<PointT, 2>{
        static float get(const PointT& p){
            return p.z;
        }
    };
}

template <int D, typename PointT>
inline float get(const PointT& p){
    return Utils::access<PointT, D>::get(p);
}

template<typename PointT, typename ContainerT = std::vector<PointT> >
class Octree{
    struct OCTREE_PARAMS{
    public:
        OCTREE_PARAMS(uint32_t buketSize = 32, bool copyPoints = false, float minExtent = 0.0f):
                    buket_size(buketSize), copy_points(copyPoints), min_extent(minExtent){}
        uint32_t buket_size;
        bool copy_points;
        float min_extent;
    };

public:
    Octree();
    ~Octree();

    //!brief initialize octree with all points.
    void initialize(const ContainerT& pts, const OCTREE_PARAMS& params = OCTREE_PARAMS());

    //!@brief initialize octree only from pts that are inside indexes.
    void initialize(const ContainerT& pts, const std::vector<uint32_t>& indexes, const OCTREE_PARAMS& params = OCTREE_PARAMS());

    //!@brief remove all data inside the octree.
    void clear();

    //!@brief radius neighbor queries where radius determines the maximal radius of reported indices of points in resultIndices.
    template<typename Distance>
    void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices) const;

    //!@brief radius neighbor queries with explicit (squared) distance computation.
    template<typename Distance>
    void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices, std::vector<float>& distances) const;

    //!@brief nearest neighbor queries, Using minDistance >= 0, we explicitly disallow self-mathes
    //!@return index of nearest neighbor n with Distance::compute(query, n) > minDistance and otherwise -1;
    template<typename Distance>
    int32_t findNeighbor(const PointT& query, float minDistance = -1) const;

protected:
    class octree_node{
    public:
        octree_node();
        ~octree_node();

        bool isLeaf;

        // bounding box of the octant neeeded for overlap and contains tests...
        float x, y, z;
        float extent;       // half of side-length

        uint32_t start, end;    // Start and end in succ_
        uint32_t size;          // number of points.
        octree_node* child[8];
    };
    Octree(Octree&);
    Octree& operator=(const Octree& oct);

    octree_node* createNode(float x, float y, float z, float extent, uint32_t startIdx, uint32_t endIdx, uint32_t size);

    //!@return true if search finished, otherwise false.
    template<typename Distance>
    bool findNeighbor(const octree_node* octant, const PointT& query, float minDistance, float maxDistance, int32_t& resultIdex) const;

    template<typename Distance>
    void radiusNeighbors(const octree_node* octant, const PointT& query, float minDistance, float& maxDistance, int32_t& resultIndex) const;

    template<typename Distance>
    void radiusNeighBors(const octree_node* octant, const PointT& query, float radius, float sqrRadius,
                         std::vector<uint32_t>& resultIndices, std::vector<float>& distance) const;

    //!@brief test if search ball S(q, r) overlaps with octant.
    static bool overlaps(const PointT& query, float radius, float sqRadius, const octree_node* o);

    OCTREE_PARAMS params_;
    octree_node* root_;
    const ContainerT* data_;

    std::vector<uint32_t> successor_;
};


template<typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::octree_node::octree_node():isLeaf(true), x(0.0f), y(0.0f), z(0.0f), extent(0.0f), start(0), end(0), size(0){
    memset(&child, 0, 8 * sizeof(octree_node));
}

template<typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::octree_node::~octree_node(){
    for(uint32_t i = 0; i < 8; ++i) delete child[i];
}

template<typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octree():root_(0), data_(0){}

template<typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::~Octree(){
    delete root_;
    if(params_.copy_points) delete data_;
}

template<typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT& pts, const OCTREE_PARAMS& params){
    clear();
    params_ = params;

    if(params_.copy_points)
        data_ = new ContainerT(pts);
    else
        data_ = &pts;

    const uint32_t N = pts.size();
    successor_ = std::vector<uint32_t>(N);

    // determine axis-aligned bounding box.
    float min[3], max[3];
    min[0] = get<0>(pts[0]);
    min[1] = get<1>(pts[0]);
    min[2] = get<2>(pts[0]);
    max[0] = min[0];
    max[1] = min[1];
    max[2] = min[2];

    for(uint32_t i = 0; i < N; ++i) {
        // initially each element links simply to the following elements.
        successor_[i] = i + 1;
        const PointT& p = pts[i];
        if(get<0>(p) < min[0]) min[0] = get<0>(p);
        if(get<1>(p) < min[1]) min[1] = get<1>(p);
        if(get<2>(p) < min[2]) min[2] = get<2>(p);
        if(get<0>(p) > max[0]) max[0] = get<0>(p);
        if(get<1>(p) > max[1]) max[1] = get<1>(p);
        if(get<2>(p) > max[2]) max[2] = get<2>(p);
    }
    float ctr[3] = {min[0], min[1], min[2]};
    float maxExtent = 0.5f * (max[0] - min[0]);

    ctr[0] += maxExtent;
    for(uint32_t i = 1; i < 3; ++i){
        float extent = 0.5f * (max[0] - min[0]);
        ctr[i] += extent;
        if(extent > maxExtent) maxExtent = extent;
    }
    root_ = createNode(ctr[0], ctr[1], ctr[2], maxExtent, 0, N-1, N);
}

template<typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT &pts, const std::vector<uint32_t> &indexes,
                                            const Octree<PointT, ContainerT>::OCTREE_PARAMS &params) {
    clear();
    params_ = params;

    if(params_.copy_points)
        data_ = new ContainerT(pts);
    else
        data_ = &pts;

    const uint32_t N = pts.size();
    successor_ = std::vector<uint32_t>(N);

    if(indexes.size() == 0) return;

    // determine axis-aligned bounding box.
    uint32_t lastIdx = indexes[0];
    float min[3], max[3];
    min[0] = get<0>(pts[lastIdx]);
    min[1] = get<1>(pts[lastIdx]);
    min[2] = get<2>(pts[lastIdx]);
    max[0] = min[0];
    max[1] = min[1];
    max[2] = min[2];

    for(uint32_t i = 1; i < indexes.size(); ++i){
        uint32_t idx = indexes[i];
        // initially each element links simply to the following element.
        successor_[lastIdx] = idx;

        const PointT& p = pts[idx];

        if(get<0>(p) < min[0]) min[0] = get<0>(p);
        if(get<1>(p) < min[1]) min[1] = get<1>(p);
        if(get<2>(p) < min[2]) min[2] = get<2>(p);
        if(get<0>(p) > max[0]) max[0] = get<0>(p);
        if(get<1>(p) > max[1]) max[1] = get<1>(p);
        if(get<2>(p) > max[2]) max[2] = get<2>(p);

        lastIdx = idx;
    }

    float ctr[3] = {min[0],min[1],min[2]};
    float maxExtent = 0.5f * (max[0] - min[0]);
    ctr[0] += maxExtent;
    for(uint32_t i = 1; i < 3; ++i){
        float extent = 0.5f * (max[i] - min[i]);
        ctr[i] += extent;
        if(extent > maxExtent) maxExtent = extent;
    }
    root_ = createNode(ctr[0], ctr[1], ctr[2], maxExtent, indexes[0], lastIdx, indexes.size());
}

template<typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::clear(){
    delete root_;
    if(params_.copy_points) delete data_;
    root_ = 0;
    data_ = 0;
    successor_.clear();
}



#endif //BASIC_ICP_QUICK_OCTREE_H
