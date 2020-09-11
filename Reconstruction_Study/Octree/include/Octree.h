//
// Created by Robotics_qi on 2020/9/4.
//

#ifndef RECONSTRUCTION_STUDY_OCTREE_H
#define RECONSTRUCTION_STUDY_OCTREE_H

#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// PCL Type.
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;
using PointCloudConstPtr = PointCloudType::ConstPtr;

namespace geometry{
    class PointCloud;
    class VoxelGrid;

    class Octree {

    public:
        /*********************************************
         * !@brief OctreeNode.
         *********************************************/

        class OctreeNode {
        public:
            //!@brief Default Constructor.
            OctreeNode() : origin_(0, 0, 0), size_(0), depth_(0), child_index_(0) {}

            /**********************************************************************
             * !@brief Parameterized Constructor.
             *
             * !@param origin coordinate of the node.
             * !@param size Size of the node.
             * !@param depth Depth of the node to the root. The root is of depth 0.
             * !@param child_index Node's child index of itself
             ***********************************************************************/
            OctreeNode(const Eigen::Vector3d &origin, const double &size, const size_t &depth,
                       const size_t &child_index) : origin_(origin), size_(size), depth_(depth),
                                                    child_index_(child_index) {

            }

            ~OctreeNode() {}

        public:

            // the node is leaf or not.
            bool isLeaf_;

            // origin coordinate of the node.
            Eigen::Vector3d origin_;
            // size of the node.
            double size_;

            // depth of the node to the root. The depth of root is 0.
            size_t depth_;

            // Node's child index of itself.
            size_t child_index_;

            // Node's child index of itself. For non-root nodes, child_index is 0~7.
            // root node's child_index is -1.
            // Children node ordering conventions are as follows.
            // root_node: origin == (0, 0, 2), size == 2.
            // Then.
            // - children_[0]: origin == (0, 0, 0), size == 1
            // - children_[1]: origin == (1, 0, 0), along X-axis next to child 0.
            // - children_[2]: origin == (0, 1, 0), along Y-axis next to child 0.
            // - children_[3]: origin == (1, 1, 0), along X-Y plane.
            // - children_[4]: origin == (0, 0, 1), along Z-axis next
            // - children_[5]: origin == (1, 0, 1), in X-Z plane.
            // - children_[6]: origin == (0, 1, 1), in Y-Z plane.
            // - children_[7]: origin == (1, 1, 1), furthest from child 0.
            std::vector<std::shared_ptr<OctreeNode> > children_;
        };



    public:
        //!@brief Default Constructor.
        Octree():
            origin_(0, 0, 0),
            size_(0),
            max_depth_(0){}

        //!@brief Parametrized Constructor.
        Octree(const size_t &max_depth)
            : origin_(0, 0, 0),
            size_(0), max_depth_(max_depth){}

        //!@brief Parametrized Constructor.
        Octree(const size_t& max_depth, const Eigen::Vector3d& origin, const double& size):
            origin_(origin), size_(size), max_depth_(max_depth){}

        Octree(const Octree& src_octree);

        ~Octree(){}

    public:
        Octree& Clear();
        bool IsEmpty() const;
        Eigen::Vector3d GetMinBound() const;
        Eigen::Vector3d GetMaxBound() const;
        Eigen::Vector3d GetCenter() const;
        Octree& Transform(const Eigen::Vector3d& transformation);
        Octree& Translate(const Eigen::Vector3d& translation);
        Octree& Scale(const double scale, const Eigen::Vector3d& center);
        Octree& Rotate(const Eigen::Matrix3d& R, const Eigen::Vector3d& translate);
        std::shared_ptr<Octree::OctreeNode> NextNode(std::shared_ptr<OctreeNode>& point_node, const Eigen::Vector3d& point);

    public:
        /************************************************
         * !@brief Construct Octree From Point Cloud.
         * !@param Input Point Cloud.
         * !@param Size expand.
         ************************************************/
         void ConstructFromPointCloud(const PointCloudConstPtr point_cloud, double size_expand = 0.01);

         // root of the octree.
         std::shared_ptr<OctreeNode> root_node_ = nullptr;

         Eigen::Vector3d origin_;

         //!@brief bounding box edge size of the whole octree.
         // A point is bound if origin_ <= point < origin_ + size_.
         double size_;

         //!@brief max depth of the octree. The depth is defined as the distance from the deepest leaf node to root.
         // A tree with only the root node has depth 0.
         size_t max_depth_;

    public:
         //!@brief Insert a point to the octree.
         //!@param point coordinate of the points.
         void InsertPoint(const Eigen::Vector3d &point);

        //!@brief DFS traversal of Octree from root.
        void Traverse();

        //!@brief Returns true if the Octree is completely the same, used for testing.
        bool operator==(const Octree& other) const;

        // Convert to Voxel Grid.
        std::shared_ptr<VoxelGrid> CovertToVoxelGrid() const;

        // Convert from Voxel Grid.
        void CreateFromVoxelGrid(const VoxelGrid& voxel_grid);

        // return true if point with bound.
        bool IsPointInBound(const Eigen::Vector3d& point, const Eigen::Vector3d& origin, const double& size);

    private:
        void TraverseRecurse();

        void InsertPointRecurse();
    };

}

#endif //RECONSTRUCTION_STUDY_OCTREE_H
