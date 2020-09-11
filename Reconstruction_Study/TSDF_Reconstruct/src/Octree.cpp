//
// Created by Robotics_qi on 2020/9/5.
//

#include <Octree.h>
#include <json/json.h>
#include <Eigen/Dense>
#include <algorithm>
#include <unordered_map>
#include <Console.h>


namespace TSDF{
    namespace geometry {
        std::shared_ptr<OctreeNode> OctreeNode::ConstructFromJsonValue(const Json::Value &value) {
            // Construct node from class name.
            std::string class_name = value.get("class_name", "").asString();
            std::shared_ptr<OctreeNode> node = nullptr;
            if (value != Json::nullValue && class_name != "") {
                if (class_name == "OctreeInternalNode")
                    node = std::make_shared<OctreeInternalNode>();
                else if (class_name == "OctreeColorLeafNode")
                    node = std::make_shared<OctreeColorLeafNode>();
                else
                    std::cerr << "Un handled class name: " << class_name << std::endl;
            }
            // Convert from json.
            if (node != nullptr) {
                bool convert_success = node->ConvertFromJsonValue(value);
                if (!convert_success) node = nullptr;
            }
            return node;
        }

        bool OctreeInternalNode::ConvertFromJsonValue(const Json::Value &value) {
            if (!value.isObject()) {
                std::cerr << "Convert From Json Value Fault !" << std::endl;
                return false;
            }

            std::string class_name = value.get("class_name", "").asString();
            if (class_name != "OctreeInternalNode") {
                std::cerr << "Failed to create OctreeInternalNode!" << std::endl;
                return false;
            }
            bool rc = true;
            for (int cid = 0; cid < 8; ++cid) {
                const auto &child_value = value["children"][Json::ArrayIndex(cid)];
                children_[cid] = OctreeNode::ConstructFromJsonValue(child_value);
            }

            return rc;
        }

        bool OctreeColorLeafNode::ConvertFromJsonValue(const Json::Value &value) {
            if (!value.isObject()) {
                std::cerr << "OctreeColor Leaf Node read Json Failed! unsupported json format." << std::endl;
                return false;
            }
            if (value.get("class_name", "") != "OctreeColorLeafNode")
                return false;
            return EigenVector3dFromJsonArray(color_, value["color"]);
        }

        std::shared_ptr<OctreeNodeInfo> OctreeInternalNode::GetInsertionNodeInfo(
                const std::shared_ptr<OctreeNodeInfo> &node_info,
                const Eigen::Vector3d &point) {
            if (!Octree::IsPointInBound(point, node_info->origin_, node_info->size_)) {
                utility::LogError("Cannot insert to child since point not in parent node bound.");
            }

            double child_size = node_info->size_ / 2.0;
            size_t x_index = point(0) < node_info->origin_(0) + child_size ? 0 : 1;
            size_t y_index = point(1) < node_info->origin_(1) + child_size ? 0 : 1;
            size_t z_index = point(2) < node_info->origin_(2) + child_size ? 0 : 1;
            size_t child_index = x_index + y_index * 2 + z_index * 4;
            Eigen::Vector3d child_origin = node_info->origin_ +
                                           Eigen::Vector3d(x_index * child_size, y_index * child_size,
                                                           z_index * child_size);
            auto child_node_info = std::make_shared<OctreeNodeInfo>(child_origin, child_size, node_info->depth_ + 1,
                                                                    child_index);
            return child_node_info;
        }

        bool OctreeInternalNode::ConvertToJsonValue(Json::Value &value) const {
            bool rc = true;
            value["class_name"] = "OctreeInternalNode";
            value["children"] = Json::arrayValue;
            value["children"].resize(8);
            for (int cid = 0; cid < 8; ++cid) {
                if (children_[cid] == nullptr)
                    value["children"][Json::ArrayIndex(cid)] = Json::objectValue;
                else
                    rc = rc && children_[cid]->ConvertToJsonValue(
                            value["children"][Json::ArrayIndex(cid)]);
            }
            return rc;
        }

        std::function<std::shared_ptr<OctreeLeafNode>()>
        OctreeColorLeafNode::GetInitFunction() {
            return []() -> std::shared_ptr<geometry::OctreeLeafNode> {
                return std::make_shared<geometry::OctreeColorLeafNode>();
            };
        }

        std::function<void(std::shared_ptr<OctreeLeafNode>)>
        OctreeColorLeafNode::GetUpdateFunction(const Eigen::Vector3d &color) {
            return [color](std::shared_ptr<geometry::OctreeLeafNode> node) -> void {
                if (auto color_leaf_node = std::dynamic_pointer_cast<geometry::OctreeColorLeafNode>(node))
                    color_leaf_node->color_ = color;
                else
                    utility::LogError("Internal error: leaf node must be OctreeLeafNode !");
            };
        }

        std::shared_ptr<OctreeLeafNode> OctreeColorLeafNode::Clone() const {
            auto cloned_node = std::make_shared<OctreeColorLeafNode>();
            cloned_node->color_ = color_;
            return cloned_node;
        }

        bool OctreeColorLeafNode::operator==(const OctreeLeafNode &that) const {
            try {
                const OctreeColorLeafNode &that_color_node = dynamic_cast<const OctreeColorLeafNode &>(that);
                return this->color_.isApprox(that_color_node.color_);
            } catch (const std::exception &) {
                return false;
            }
        }

        bool OctreeColorLeafNode::ConvertToJsonValue(Json::Value &value) const {
            value["class_name"] = "OctreeColorLeafNode";
            return EigenVector3dToJsonArray(color_, value["color"]);
        }

        Octree::Octree(const Octree &src_octree) :
                origin_(src_octree.origin_),
                size_(src_octree.size_),
                max_depth_(src_octree.max_depth_) {
            // First traversal: clone nodes without edges.
            std::unordered_map<std::shared_ptr<OctreeNode>, std::shared_ptr<OctreeNode>> map_src_to_dst_node;
            auto f_build_map = [&map_src_to_dst_node](const std::shared_ptr<OctreeNode> &src_node,
                                                      const std::shared_ptr<OctreeNodeInfo> &src_node_info) -> void {
                if (auto src_internal_node = std::dynamic_pointer_cast<OctreeInternalNode>(src_node)) {
                    auto dst_internal_node = std::make_shared<OctreeInternalNode>();
                    map_src_to_dst_node[src_internal_node] = dst_internal_node;
                } else if (auto src_leaf_node = std::dynamic_pointer_cast<OctreeLeafNode>(src_node)) {
                    map_src_to_dst_node[src_leaf_node] = src_leaf_node->Clone();
                } else {
                    utility::LogError("Internal error: unkown node type.");
                }
            };
            src_octree.Traverse(f_build_map);

            // Second traversal: add edges.
            auto f_clone_edge = [&map_src_to_dst_node](const std::shared_ptr<OctreeNode> &src_node,
                                                       const std::shared_ptr<OctreeNodeInfo> &src_node_info) -> void {
                if (auto src_internal_node = std::dynamic_pointer_cast<OctreeInternalNode>(src_node)) {
                    auto dst_internal_node = std::dynamic_pointer_cast<OctreeInternalNode>(
                            map_src_to_dst_node.at(src_internal_node));

                    for (size_t child_index = 0; child_index < 8; child_index++) {
                        auto src_child_node = src_internal_node->children_[child_index];
                        if (src_child_node != nullptr) {
                            auto dst_child_node = map_src_to_dst_node.at(src_child_node);
                            dst_internal_node->children_[child_index] = dst_child_node;
                        }
                    }
                }
            };
            src_octree.Traverse(f_clone_edge);

            root_node_ = map_src_to_dst_node.at(src_octree.root_node_);
        }

        bool Octree::IsPointInBound(const Eigen::Vector3d &point, const Eigen::Vector3d &origin, const double &size) {
            bool rc = (Eigen::Array3d(origin) <= Eigen::Array3d(point)).all() &&
                      (Eigen::Array3d(point) < Eigen::Array3d(origin) + size).all();
            return rc;
        }

        void Octree::Traverse(
                const std::function<void(const std::shared_ptr<OctreeNode> &,
                                         const std::shared_ptr<OctreeNodeInfo> &)> &f) {
            TraverseRecurse(root_node_, std::make_shared<OctreeNodeInfo>(origin_, size_, 0, 0), f);
        }

        void Octree::Traverse(
                const std::function<void(const std::shared_ptr<OctreeNode>&, const std::shared_ptr<OctreeNodeInfo>&)>& f) const{
            TraverseRecurse(root_node_, std::make_shared<OctreeNodeInfo>(origin_, size_, 0, 0), f);
        }

        void Octree::TraverseRecurse(const std::shared_ptr<OctreeNode> &node,
                                     const std::shared_ptr<OctreeNodeInfo> &node_info,
                                     const std::function<void(const std::shared_ptr<OctreeNode> &,
                                                              const std::shared_ptr<OctreeNodeInfo> &)> &f) {
            if(node == nullptr)
                return;
            else if(auto internal_node = std::dynamic_pointer_cast<OctreeInternalNode>(node)){
                f(internal_node, node_info);
                double child_size = node_info->size_ / 2.0;

                for(size_t child_index = 0; child_index < 8; ++child_index){
                    size_t x_index = child_index % 2;
                    size_t y_index = (child_index / 2) % 2;
                    size_t z_index = (child_index / 4) % 2;

                    auto child_node = internal_node->children_[child_index];
                    Eigen::Vector3d child_node_origin = node_info->origin_ + Eigen::Vector3d(double(x_index), double(y_index), double(z_index)) * child_size;
                    auto child_node_info = std::make_shared<OctreeNodeInfo>(child_node_origin, child_size, node_info->depth_ + 1, child_index);
                    TraverseRecurse(child_node, child_node_info, f);
                }
            }else if(auto leaf_node = std::dynamic_pointer_cast<OctreeLeafNode>(node)){
                f(leaf_node, node_info);
            }else{
                utility::LogError("Internal error: unknown node type.");
            }
        }

        void Octree::InsertPoint(const Eigen::Vector3d &point,
                                 const std::function<std::shared_ptr<OctreeLeafNode>()> &f_init,
                                 const std::function<void(std::shared_ptr<OctreeLeafNode>)> &f_update) {
            if(root_node_ == nullptr){
                if(max_depth_ == 0)
                    root_node_ = f_init();
                else
                    root_node_ = std::make_shared<OctreeInternalNode>();
            }
            auto root_node_info = std::make_shared<OctreeNodeInfo>(origin_, size_, 0, 0);
            InsertPointRecurse(root_node_, root_node_info, point, f_init, f_update);
        }

        void Octree::InsertPointRecurse(const std::shared_ptr<OctreeNode> &node,
                                        const std::shared_ptr<OctreeNodeInfo> &node_info, const Eigen::Vector3d &point,
                                        const std::function<std::shared_ptr<OctreeLeafNode>()> &f_init,
                                        const std::function<void(std::shared_ptr<OctreeLeafNode>)> &f_update) {
            if(!IsPointInBound(point, node_info->origin_, node_info->size_)) return;

            if(node_info->depth_ > max_depth_)
                return;
            else if(node_info->depth_ == max_depth_){
                if(auto leaf_node = std::dynamic_pointer_cast<OctreeLeafNode>(node))
                    f_update(leaf_node);
                else
                    utility::LogError("Internal Error: Leaf Node Must Be OctreeLeafNode.");
            }else{
                if(auto internal_node = std::dynamic_pointer_cast<OctreeInternalNode>(node)){
                    // Get Child node info.
                    std::shared_ptr<OctreeNodeInfo> child_node_info = internal_node->GetInsertionNodeInfo(node_info, point);

                    // Create child node with factory function.
                    size_t child_index = child_node_info->child_index_;
                    if(internal_node->children_[child_index] == nullptr){
                        if(node_info->depth_ == max_depth_ - 1)
                            internal_node->children_[child_index] = f_init();
                        else
                            internal_node->children_[child_index] = std::make_shared<OctreeInternalNode>()
                    }
                    InsertPointRecurse(internal_node->children_[child_index], child_node_info, point, f_init, f_update);
                }else{
                    utility::LogError("Internal Error: Internal node must be OctreeInternalNode.");
                }
            }
        }

        void Octree::ConvertFromPointCloud(const PointCloudPtr point_cloud, double size_expand){
            if(size_expand > 1 || size_expand < 0)
                utility::LogError("size_expand shall be between 0 and 1");

            //
        }
    }
}