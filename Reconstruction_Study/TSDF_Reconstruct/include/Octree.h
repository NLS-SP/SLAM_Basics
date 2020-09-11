#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "IJsonConvertible.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// PCL Type.
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;
using PointCloudConstPtr = PointCloudType::ConstPtr;

namespace TSDF{
    namespace geometry {
        class PointCloud;

        class VoxelGrid;

        class OctreeNodeInfo {
        public:
            OctreeNodeInfo() : origin_(0, 0, 0), size_(0), depth_(0), child_index_(0) {}

            OctreeNodeInfo(const Eigen::Vector3d &origin, const double &size, const size_t &depth,
                           const size_t &child_index) :
                    origin_(origin), size_(size), depth_(depth), child_index_(child_index) {}

            ~OctreeNodeInfo() {}

        public:
            Eigen::Vector3d origin_;
            double size_;
            size_t depth_;
            size_t child_index_;
        };

        class OctreeNode : public utility::IJsonConvertible {
        public:
            // !@brief
            OctreeNode() {}

            virtual ~OctreeNode() {}

            static std::shared_ptr<OctreeNode> ConstructFromJsonValue(const Json::Value &value);
        };

        class OctreeLeafNode : public OctreeNode {
        public:
            virtual bool operator==(const OctreeLeafNode &other) const = 0;

            virtual std::shared_ptr<OctreeLeafNode> Clone() const = 0;
        };

        class OctreeColorLeafNode : public OctreeLeafNode {
        public:

            bool operator==(const OctreeLeafNode &other) const override;

            std::shared_ptr<OctreeLeafNode> Clone() const override;

            static std::function<std::shared_ptr<OctreeLeafNode>()> GetInitFunction();

            static std::function<void(std::shared_ptr<OctreeLeafNode>)> GetUpdateFunction(const Eigen::Vector3d &color);

            bool ConvertToJsonValue(Json::Value &value) const override;

            bool ConvertFromJsonValue(const Json::Value &value) override;

            Eigen::Vector3d color_ = Eigen::Vector3d(0, 0, 0);
        };

        class OctreeInternalNode : public OctreeNode {
        public:
            OctreeInternalNode() : children_(8) {}

            static std::shared_ptr<OctreeNodeInfo>
            GetInsertionNodeInfo(const std::shared_ptr<OctreeNodeInfo> &node_info, const Eigen::Vector3d &point);

            bool ConvertToJsonValue(Json::Value &value) const override;

            bool ConvertFromJsonValue(const Json::Value &value) override;

        public:
            std::vector<std::shared_ptr<OctreeNode> > children_;
        };

        class Octree : public utility::IJsonConvertible {
        public:
            Octree() :
                    origin_(0, 0, 0),
                    size_(0),
                    max_depth_(0) {}

            Octree(const size_t &max_depth) :
                    origin_(0, 0, 0),
                    size_(0),
                    max_depth_(max_depth) {}

            Octree(const size_t &max_depth, const Eigen::Vector3d &origin,
                   const double &size) :
                    origin_(origin),
                    size_(size),
                    max_depth_(max_depth) {}

            Octree(const Octree &src_octree);

            ~Octree() {}

        public:
            void ConvertFromPointCloud(const PointCloudPtr point_cloud, double size_expand = 0.01);

            // Root of the octree.
            std::shared_ptr<OctreeNode> root_node_ = nullptr;

            Eigen::Vector3d origin_;

            double size_;

            size_t max_depth_;

            void InsertPoint(const Eigen::Vector3d &point,
                             const std::function<std::shared_ptr<OctreeLeafNode>()> &f_init,
                             const std::function<void(std::shared_ptr<OctreeLeafNode>)> &f_update);

            void Traverse(const std::function<void(const std::shared_ptr<OctreeNode> &,
                                                   const std::shared_ptr<OctreeNodeInfo> &)> &f);

            void Traverse(const std::function<void(const std::shared_ptr<OctreeNode> &,
                                                   const std::shared_ptr<OctreeNodeInfo> &)> &f) const;

            std::pair<std::shared_ptr<OctreeLeafNode>, std::shared_ptr<OctreeNodeInfo> >
            LocateLeafNode(const Eigen::Vector3d &point) const;

            static bool IsPointInBound(const Eigen::Vector3d &point, const Eigen::Vector3d &origin, const double &size);

            bool operator==(const Octree &other) const;

            // Convert to VoxelGrid.
            std::shared_ptr<geometry::VoxelGrid> ToVoxelGrid() const;

            // Convert from voxel grid.
            void CreateFromVoxelGrid(const geometry::VoxelGrid &voxel_grid);

        private:
            static void TraverseRecurse(const std::shared_ptr<OctreeNode> &node,
                                        const std::shared_ptr<OctreeNodeInfo> &node_info,
                                        const std::function<void(const std::shared_ptr<OctreeNode> &,
                                                                 const std::shared_ptr<OctreeNodeInfo> &)> &f);

            void InsertPointRecurse(const std::shared_ptr<OctreeNode> &node,
                                    const std::shared_ptr<OctreeNodeInfo> &node_info,
                                    const Eigen::Vector3d &point,
                                    const std::function<std::shared_ptr<OctreeLeafNode>()> &f_init,
                                    const std::function<void(std::shared_ptr<OctreeLeafNode>)> &f_update);
        };
    }
}