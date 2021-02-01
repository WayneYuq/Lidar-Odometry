/*
 * @Description: MYNDT 
 * @Author: Yu Qianhai
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/my_ndt_registration.hpp"
#include "lidar_localization/models/registration/ndt_omp.h"
#include "lidar_localization/models/registration/ndt_omp_impl.hpp"
#include "lidar_localization/models/registration/voxel_grid_covariance_omp.hpp"
#include "lidar_localization/models/registration/voxel_grid_covariance_omp_impl.hpp"

#include "glog/logging.h"

namespace lidar_localization {

MYNDTRegistration::MYNDTRegistration(const YAML::Node& node)
    :myndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

MYNDTRegistration::MYNDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :myndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool MYNDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter, pclomp::NeighborSearchMethod sear_methods) {
    myndt_ptr_->setResolution(res);
    myndt_ptr_->setStepSize(step_size);
    myndt_ptr_->setTransformationEpsilon(trans_eps);
    myndt_ptr_->setMaximumIterations(max_iter);
    myndt_ptr_->setNeighborhoodSearchMethod(sear_methods);

    LOG(INFO) << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool MYNDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    myndt_ptr_->setInputTarget(input_target);

    return true;
}

bool MYNDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    myndt_ptr_->setInputSource(input_source);
    myndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = myndt_ptr_->getFinalTransformation();

    return true;
}
}