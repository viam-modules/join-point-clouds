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

#include "utils.h"

using namespace viam::sdk;

Eigen::Vector3f parseTranslation(const google::protobuf::ListValue& list_value) {
    if (list_value.values_size() != 3) {
        throw std::invalid_argument("translation must have exactly 3 elements.");
    }
    Eigen::Vector3f translation;
    for (int i = 0; i < 3; ++i) {
        if (!list_value.values(i).has_number_value()) {
            throw std::invalid_argument("all elements of translation must be numeric.");
        }
        translation(i) = list_value.values(i).number_value();
    }
    return translation;
}

Eigen::Quaternionf parseQuaternion(const google::protobuf::ListValue& list_value) {
    if (list_value.values_size() != 4) {
        throw std::invalid_argument("quaternion must have exactly 4 elements.");
    }
    Eigen::Quaternionf quaternion(
        list_value.values(3).number_value(), // w
        list_value.values(0).number_value(), // x
        list_value.values(1).number_value(), // y
        list_value.values(2).number_value()  // z
    );
    return quaternion;
}

pcl::PointCloud<pcl::PointXYZ> combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds) {
    pcl::PointCloud<pcl::PointXYZ> combinedCloud;
    for (const auto& pcd : clouds) {
        combinedCloud += *pcd;
    }
    return combinedCloud;
}

class JoinPointClouds : public Camera, public Reconfigurable {
   public:
    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override {
        std::cout << "join-point-clouds " << Resource::name() << " is reconfiguring\n";
        camTransformPairs = {};  // reset to empty

        std::vector<std::shared_ptr<Camera>> cams;
        for (auto& dep : deps) {
            // TODO: Refactor to getCams helper vvv
            std::shared_ptr<Resource> cam_resource = dep.second;
            if (cam_resource->api().to_string() != API::get<Camera>().to_string()) {
                std::ostringstream buffer;
                buffer << "dependency " << cam_resource->api().to_string()
                       << " is not a camera resource\n";
                throw std::invalid_argument(buffer.str());
            }
            auto cam = std::dynamic_pointer_cast<Camera>(cam_resource);
            if (!cam->get_properties().supports_pcd) {
                std::ostringstream buffer;
                buffer << "camera resource " << cam_resource->api().to_string()
                       << " does not support get_point_cloud\n";
                throw std::invalid_argument(buffer.str());
            }
            std::cout << "camera " << cam->name() << " registered\n";
            // TODO: Refactor to getCams helper ^^^
            cams.push_back(cam);
        }

        std::vector<std::shared_ptr<Eigen::Matrix4f>> transformations;
        auto attrs = cfg.attributes();
        // TODO: Refactor to getTransformations helper vvv  input: attrs;  output: transformations / throws error
        if (attrs->count("transforms") == 1) {
            std::shared_ptr<ProtoType> transforms_proto = attrs->at("transforms");
            auto transforms_value = transforms_proto->proto_value();

            if (!transforms_value.has_list_value()) {
                throw std::invalid_argument("'transforms' field must be a list");
            }
            auto transforms_list = transforms_value.list_value();

            if (transforms_list.values().size() != cams.size()) {
                throw std::invalid_argument(
                    "'transforms' list must contain the same number of elements as cams");
            }

            for (const auto& transform_value : transforms_list.values()) {
                if (!transform_value.has_list_value()) {
                    throw std::invalid_argument("each item in 'transforms' must be a list");
                }
                auto transform = transform_value.list_value();
                if (transform.values().size() != 2) {
                    throw std::invalid_argument("each transform must consist of two lists (translation, quaternion)");
                }

                Eigen::Vector3f translation = parseTranslation(transform.values(0).list_value());
                Eigen::Quaternionf quaternion = parseQuaternion(transform.values(1).list_value());

                std::cout << "Translation: " << translation.transpose() << std::endl;
                std::cout << "Quaternion: " << quaternion.coeffs().transpose() << std::endl;

                Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                transformation.block<3,3>(0,0) = quaternion.toRotationMatrix();
                transformation.block<3,1>(0,3) = translation;
                transformations.push_back(std::make_shared<Eigen::Matrix4f>(transformation));

                std::cout << "Transformation Matrix:" << std::endl;
                std::cout << *transformations.back() << std::endl;
            }
        } else if (attrs->count("transforms") > 1) {
            throw std::invalid_argument(
                "required 'transforms' attribute in the config was specified more than once");
        } else {
            for (auto& cam : cams) {
                std::shared_ptr<Eigen::Matrix4f> identityMatrix = std::make_shared<Eigen::Matrix4f>(Eigen::Matrix4f::Identity());
                transformations.push_back(identityMatrix);
            }
            std::cout << "no 'transforms' attribute specified; using world coordinates\n";
        }
        // TODO: Refactor to getTransformations ^^^

        for (size_t i = 0; i < cams.size(); ++i) {
            camTransformPairs.push_back(std::make_pair(cams[i], transformations[i]));
        }
        std::cout << "size of camTransformPairs: " << camTransformPairs.size() << std::endl;

        // TODO: Refactor to getTargetFrame helper vvv
        if (attrs->count("target_frame") == 1) {
            std::shared_ptr<ProtoType> target_frame_proto = attrs->at("target_frame");
            auto target_frame_value = target_frame_proto->proto_value();

            if (target_frame_value.has_string_value()) {
                targetFrame = target_frame_value.string_value();
                if (targetFrame != "world") {
                    bool isValidTargetFrame = false;
                    for (auto cam : cams) {
                        if (targetFrame == cam->name()) {
                            isValidTargetFrame = true;
                            break;
                        }
                    }
                    if (!isValidTargetFrame) {
                        throw std::invalid_argument("'target_frame' field must be a source camera component name");
                    }
                }
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
    }

    JoinPointClouds(Dependencies deps, ResourceConfig cfg) : Camera(cfg.name()) {
        std::cout << "Creating join-point-clouds " + Resource::name() << std::endl;
        reconfigure(deps, cfg);
    }

    AttributeMap do_command(const AttributeMap& command) override {
        std::ostringstream buffer;
        buffer << "do_command method is unsupported\n";
        throw std::runtime_error(buffer.str());
    }

    std::vector<GeometryConfig> get_geometries(const AttributeMap& extra) override {
        return std::vector<GeometryConfig>();
    }

    raw_image get_image(std::string mime_type, const AttributeMap& extra) override {
        std::ostringstream buffer;
        buffer << "get_image method is unsupported\n";
        throw std::runtime_error(buffer.str());
    }

    image_collection get_images() override {
        std::ostringstream buffer;
        buffer << "get_images method is unsupported\n";
        throw std::runtime_error(buffer.str());
    }

    point_cloud get_point_cloud(std::string mime_type, const AttributeMap& extra) {
        std::cout << "in 'get_point_cloud'" << std::endl;

        std::vector<RawPCD> rawPCDs;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
        pcl::PointCloud<pcl::PointXYZ> combinedCloud;
        for (auto camTransformPair : camTransformPairs) {
            auto cam = camTransformPair.first;
            auto response = cam->get_point_cloud("pointcloud/pcd");
            std::cout << ".size() of response.pc: " << response.pc.size() << std::endl;
            RawPCD rawPCD = parseRawPCD(response.pc);
            rawPCDs.push_back(rawPCD);
            std::cout << rawPCD << std::endl;

            auto pcd = convertToPointCloud(rawPCD);
            clouds.push_back(pcd);
            std::cout << "Cloud has " << pcd->points.size() << " points." << std::endl;
        }

        for (int i = 0; i < clouds.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = clouds[i];
            std::shared_ptr<Eigen::Matrix4f> transformation = camTransformPairs[i].second;

            Eigen::Matrix4f inverseTransformation = transformation->inverse();
            pcl::PointCloud<pcl::PointXYZ>::Ptr worldCloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*cloud, *worldCloud, inverseTransformation);
            clouds[i] = worldCloud;
        }

        if (targetFrame != "world") {
            std::shared_ptr<Eigen::Matrix4f> transformation = getTargetFrameTransformation();
            for (int i = 0; i < clouds.size(); ++i) {
                std::shared_ptr<Camera> cam = camTransformPairs[i].first;
                pcl::PointCloud<pcl::PointXYZ>::Ptr worldCloud = clouds[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*worldCloud, *targetCloud, *transformation);
                clouds[i] = targetCloud;
            }
        }

        combinedCloud = combinePointClouds(clouds);

        std::cout << "Combined cloud has " << combinedCloud.size() << " points." << std::endl;
        std::vector<unsigned char> pcd_bytes = pclCloudToPCDBytes(combinedCloud);

        return point_cloud{mime_type, pcd_bytes};
    }

    properties get_properties() override {
        std::ostringstream buffer;
        buffer << "get_properties unsupported is unsupported\n";
        throw std::runtime_error(buffer.str());
    }

   private:
    using CameraTransformationPair = std::pair<std::shared_ptr<Camera>, std::shared_ptr<Eigen::Matrix4f>>;
    std::vector<CameraTransformationPair> camTransformPairs;
    std::string targetFrame;

    std::shared_ptr<Eigen::Matrix4f> getTargetFrameTransformation() {
        for (auto pair : camTransformPairs) {
            auto& cam = pair.first;
            auto& transformation = pair.second;
            if (cam->name() == targetFrame) {
                return transformation;
            }
        }
        std::ostringstream buffer;
        buffer << "could not find target frame in source cameras\n";
        throw std::runtime_error(buffer.str());
    }
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

    return deps;
}

int main(int argc, char** argv) {
    API camera_api = API::get<Camera>();
    Model m("viam", "camera", "join-point-clouds");

    std::shared_ptr<ModelRegistration> mr = std::make_shared<ModelRegistration>(
        camera_api,
        m,
        [](Dependencies deps, ResourceConfig cfg) {
            return std::make_unique<JoinPointClouds>(deps, cfg);
        },
        validate);

    std::vector<std::shared_ptr<ModelRegistration>> mrs = {mr};
    auto my_mod = std::make_shared<ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
};
