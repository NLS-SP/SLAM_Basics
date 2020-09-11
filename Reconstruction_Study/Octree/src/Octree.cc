//
// Created by Robotics_qi on 2020/9/4.
//

#include <Octree.h>
#include <unordered_map>

namespace geometry{
    Octree::Octree(const Octree& src_octree):
            origin_(src_octree.origin_),
            size_(src_octree.size_),
            max_depth_(src_octree.max_depth_){

    }

    Octree& Octree::Clear(){
        root_node_ = nullptr;
        origin_.setZero();
        size_ = 0;
        return *this;
    }

    bool Octree::IsEmpty() const { return root_node_ == nullptr; }

    void Octree::InsertPoint(const Eigen::Vector3d &point) {
        if(root_node_ == nullptr){
            if(max_depth_ == 0)
                root_node_ = std::make_shared<OctreeNode>();
            else{
                // TODO: 这里应该是将node应该所属的子索引节点找到,返回后，将其移动到子节点
                root_node_ = NextNode(root_node_, point);
            }
        }
    }

    std::shared_ptr<Octree::OctreeNode> Octree::NextNode(std::shared_ptr<Octree::OctreeNode>& point_node, const Eigen::Vector3d& point) {
        if(IsPointInBound(point, point_node->origin_, const Eigen::Vector3d& point)){
            std::cerr << "Cannot insert to child since point not in parent node bound." << std::endl;
        }
        double child_size = point_node->size_ / 2.0;
        size_t x_index = point(0) < point_node->origin_(0) + child_size ? 0 : 1;
        size_t y_index = point(1) < point_node->origin_(1) + child_size ? 0 : 1;
        size_t z_index = point(2) < point_node->origin_(2) + child_size ? 0 : 1;
        size_t child_index = x_index + y_index * 2 + z_index * 4;
        Eigen::Vector3d child_origin = point_node->origin_ + Eigen::Vector3d(x_index * child_size, y_index * child_size, z_index * child_size);

        auto child_node = std::make_shared<Octree::OctreeNode>(child_origin, child_size, point_node->depth_ + 1, child_index);
        return child_node;
    }

    bool Octree::operator==(const Octree& other) const{
        // Check basic Properties.
        bool rc = true;
        rc = rc && origin_.isApprox(other.origin_);
        rc = rc && size_ == other.size_;
        rc = rc && max_depth_ == other.max_depth_;
        if(!rc)
            return rc;
    }
}
