#include "static_transform_alias.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/make_shared.hpp>

namespace util_nodes_tf2_ros_tool {

class StaticTransformAliasNodelet : public nodelet::Nodelet {
public:
    virtual void onInit() override;
private:
    boost::shared_ptr<StaticTransformAlias> m_;
};

void StaticTransformAliasNodelet::onInit() {
    // Create a shared pointer to the StaticTransformAlias using the node handles provided by the nodelet.
    m_ = boost::make_shared<StaticTransformAlias>(getNodeHandle(), getPrivateNodeHandle());
}

} // namespace util_nodes_tf2_ros_tool

PLUGINLIB_EXPORT_CLASS(util_nodes_tf2_ros_tool::StaticTransformAliasNodelet, nodelet::Nodelet);
