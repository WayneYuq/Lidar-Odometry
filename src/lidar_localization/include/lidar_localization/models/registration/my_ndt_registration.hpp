/*
 * @Description: MYNDT 匹配模块
 * @Author: Yu Qianhai
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_MYNDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_MYNDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/ndt_omp.h"

namespace lidar_localization {
class MYNDTRegistration: public RegistrationInterface {
  public:
    MYNDTRegistration(const YAML::Node& node);
    MYNDTRegistration(float res, float step_size, float trans_eps, int max_iter, pclomp::NeighborSearchMethod sear_methods);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter, pclomp::NeighborSearchMethod sear_methods);

  private:
    pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr myndt_ptr_;
};
}

#endif