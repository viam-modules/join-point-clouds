#include <grpcpp/grpcpp.h>
#include <grpcpp/server_context.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <signal.h>
#include <viam/api/common/v1/common.grpc.pb.h>
#include <viam/api/component/generic/v1/generic.grpc.pb.h>
#include <viam/api/robot/v1/robot.pb.h>

#include <boost/log/trivial.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/rpc/dial.hpp>
#include <viam/sdk/rpc/server.hpp>
#include <viam/sdk/services/motion.hpp>

#include "utils.h"

using namespace viam::sdk;

class JoinPointClouds : public Camera, public Reconfigurable {
   public:
    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override {
        std::cout << "join-point-clouds " << Resource::name() << " is reconfiguring\n";
        camTransformPairs = {};  // reset to empty

        std::shared_ptr<Motion> motion;
        std::vector<NamedCamera> namedCams;
        for (auto& dep : deps) {
            auto res = dep.second;
            auto api = res->api().to_string();
            if (api == API::get<Camera>().to_string()) {
                // TODO: Refactor to getCams helper vvv
                auto cam = std::dynamic_pointer_cast<Camera>(res);
                if (!cam->get_properties().supports_pcd) {
                    throw std::invalid_argument(std::string("camera resource ") +
                            api +
                            " does not support get_point_cloud\n");
                }
                std::cout << "camera " << cam->name() << " registered\n";
                // TODO: Refactor to getCams helper ^^^
                namedCams.push_back(NamedCamera{dep.first, cam});
            } else if (api == API::get<Motion>().to_string()) {
                motion = std::dynamic_pointer_cast<Motion>(res);
            } else {
                throw std::invalid_argument(std::string("dependency ") +
                           api +
                           " is not a camera resource\n");
            }
        }

        // TODO: Refactor to getTargetFrame helper vvv
        auto attrs = cfg.attributes();
        if (attrs->count("target_frame") == 1) {
            std::shared_ptr<ProtoType> targetFrameProto = attrs->at("target_frame");
            auto targetFrameVal = targetFrameProto->proto_value();

            if (targetFrameVal.has_string_value()) {
                targetFrame = targetFrameVal.string_value();
                std::cout << "target frame: " << targetFrame << std::endl;
            } else {
                throw std::invalid_argument("'target_frame' field must be a string");
            }
        } else if (attrs->count("target_frame") > 1) {
            throw std::invalid_argument(
                "required 'target_frame' attribute in the config was specified more than once");
        } else {
            throw std::invalid_argument(
                "could not find required 'target_frame' attribute in the config");
        }
        // TODO: Refactor to getTargetFrame helper ^^^

        for (auto namedCam : namedCams) {
            auto name = namedCam.first;
            std::cout << "Cam name: " << name << std::endl;
            auto cam = namedCam.second;
            std::vector<WorldState::transform> supplementalTransforms;
            pose_in_frame poseInFrame = motion->get_pose(name, targetFrame, supplementalTransforms);
            double thetaRadians = poseInFrame.pose.theta * M_PI / 180.0;
            coordinates coords = poseInFrame.pose.coordinates;
            pose_orientation ori = poseInFrame.pose.orientation;
            std::cout << "Theta (degrees): " << poseInFrame.pose.theta << std::endl;
            std::cout << "Theta (radians): " << thetaRadians << std::endl;
            std::cout << "Coordinates:" << std::endl;
            std::cout << "  x: " << coords.x << std::endl;
            std::cout << "  y: " << coords.y << std::endl;
            std::cout << "  z: " << coords.z << std::endl;

            std::cout << "Orientation:" << std::endl;
            std::cout << "  o_x: " << ori.o_x << std::endl;
            std::cout << "  o_y: " << ori.o_y << std::endl;
            std::cout << "  o_z: " << ori.o_z << std::endl;

            // Compute 3x3 rotation matrix
            ViamOrientationVector orientationVec(ori.o_x, ori.o_y, ori.o_z, thetaRadians);
            Eigen::Quaternionf quaternion = orientationVec.toQuaternion();
            Eigen::Matrix3f rotationMatrix = quaternion.toRotationMatrix();

            // Initialize 3x1 translation vector
            Eigen::Vector3f translation(coords.x, coords.y, coords.z);

            // Initialize 4x4 homogeneous transformation from current cam to target frame
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 3>(0, 0) = rotationMatrix;
            transform.block<3, 1>(0, 3) = translation;

            camTransformPairs.push_back(std::make_pair(cam, std::make_shared<Eigen::Matrix4f>(transform)));
        }
    }

    JoinPointClouds(Dependencies deps, ResourceConfig cfg) : Camera(cfg.name()) {
        std::cout << "Creating join-point-clouds " + Resource::name() << std::endl;
        reconfigure(deps, cfg);
    }

    AttributeMap do_command(const AttributeMap& command) override {
        throw std::runtime_error("do_command method is unsupported\n");
    }

    std::vector<GeometryConfig> get_geometries(const AttributeMap& extra) override {
        throw std::runtime_error("get_geometries method is unsupported\n");
    }

    raw_image get_image(std::string mime_type, const AttributeMap& extra) override {
        throw std::runtime_error("get_image method is unsupported\n");
    }

    image_collection get_images() override {
        throw std::runtime_error("get_images method is unsupported\n");
    }

    point_cloud get_point_cloud(std::string mime_type, const AttributeMap& extra) {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
        for (auto camTransformPair : camTransformPairs) {
            auto cam = camTransformPair.first;
            auto response = cam->get_point_cloud("pointcloud/pcd");
            RawPCD rawPCD = parseRawPCD(response.pc);

            auto cloud = convertToPointCloud(rawPCD);
            std::shared_ptr<Eigen::Matrix4f> transformation = camTransformPair.second;
            pcl::transformPointCloud(*cloud, *cloud, *transformation);
            clouds.push_back(cloud);
            std::cout << "Transformed cloud has " << cloud->points.size() << " points." << std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ> combinedCloud = combinePointClouds(clouds);
        std::cout << "Combined cloud has " << combinedCloud.size() << " points." << std::endl;
        std::vector<unsigned char> response = pclCloudToPCDBytes(combinedCloud);
        return point_cloud{mime_type, response};
    }

    properties get_properties() override {
        properties response;
        response.supports_pcd = true;
        return response;
    }

   private:
    using NamedCamera = std::pair<Name, std::shared_ptr<Camera>>;
    using CameraTransformationPair = std::pair<std::shared_ptr<Camera>, std::shared_ptr<Eigen::Matrix4f>>;
    std::vector<CameraTransformationPair> camTransformPairs;
    std::string targetFrame;
};

std::vector<std::string> validate(ResourceConfig cfg) {
    auto attrs = cfg.attributes();
    std::vector<std::string> deps;

    if (attrs->count("source_cameras") == 1) {
        std::shared_ptr<ProtoType> source_cameras_proto = attrs->at("source_cameras");
        auto source_cameras_value = source_cameras_proto->proto_value();

        if (source_cameras_value.has_list_value()) {
            auto source_cameras_list = source_cameras_value.list_value();

            if (source_cameras_list.values().size() == 0) {
                throw std::invalid_argument(
                    "source_cameras field cannot be empty, must list at least 1 camera name");
            }

            for (const auto& value : source_cameras_list.values()) {
                if (!value.has_string_value()) {
                    throw std::invalid_argument(
                        "each item in source_cameras list must be a string");
                }
                deps.push_back(value.string_value());
            }
        } else {
            throw std::invalid_argument("source_cameras field must be a list");
        }
    } else if (attrs->count("source_cameras") > 1) {
        throw std::invalid_argument(
            "required 'source_cameras' attribute in the config was specified more than once");
    } else {
        throw std::invalid_argument(
            "could not find required 'source_cameras' attribute in the config");
    }

    deps.push_back("rdk:service:motion/builtin");
    return deps;
}

int main(int argc, char** argv) {
    API camera_api = API::get<Camera>();
    Model m("viam", "camera", "join-point-clouds");

    std::shared_ptr<ModelRegistration> mr = std::make_shared<ModelRegistration>(
        camera_api,
        m,
        [](Dependencies deps, ResourceConfig cfg) {
            return std::make_shared<JoinPointClouds>(deps, cfg);
        },
        validate);

    std::vector<std::shared_ptr<ModelRegistration>> mrs = {mr};
    auto my_mod = std::make_shared<ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
};
