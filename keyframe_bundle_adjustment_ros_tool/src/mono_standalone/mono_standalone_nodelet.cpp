#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "mono_standalone.hpp"

namespace keyframe_bundle_adjustment_ros_tool {

class MonoStandaloneNodelet : public nodelet::Nodelet {
public:
    void onInit() override {
        // Instantiate MonoStandalone with public and private node handles.
        impl_ = std::make_unique<MonoStandalone>(getNodeHandle(), getPrivateNodeHandle());
    }

private:
    std::unique_ptr<MonoStandalone> impl_;
};

} // namespace keyframe_bundle_adjustment_ros_tool

PLUGINLIB_EXPORT_CLASS(keyframe_bundle_adjustment_ros_tool::MonoStandaloneNodelet, nodelet::Nodelet);
