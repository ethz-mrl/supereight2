/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SENSOR_IMPL_HPP
#define SE_SENSOR_IMPL_HPP

namespace se {



template<typename DerivedT>
template<typename ConfigT>
SensorBase<DerivedT>::SensorBase(const ConfigT& c) :
        left_hand_frame(false),
        near_plane(c.near_plane),
        far_plane(c.far_plane),
        T_BS(c.T_BS),
        pixel_voxel_ratio_per_scale(c.pixel_voxel_ratio_per_scale)
{
}



template<typename DerivedT>
SensorBase<DerivedT>::SensorBase(const DerivedT& d) :
        left_hand_frame(d.left_hand_frame),
        near_plane(d.near_plane),
        far_plane(d.far_plane),
        T_BS(d.T_BS),
        pixel_voxel_ratio_per_scale(d.pixel_voxel_ratio_per_scale)
{
}



template<typename DerivedT>
int SensorBase<DerivedT>::blockIntegrationScale(const Eigen::Vector3f& block_centre_S,
                                                const float map_res,
                                                const int last_scale,
                                                const int min_scale,
                                                const int max_block_scale) const
{
    return underlying()->blockIntegrationScaleImpl(
        block_centre_S, map_res, last_scale, min_scale, max_block_scale);
}



template<typename DerivedT>
float SensorBase<DerivedT>::nearDist(const Eigen::Vector3f& ray_S) const
{
    return underlying()->nearDistImpl(ray_S);
}



template<typename DerivedT>
float SensorBase<DerivedT>::farDist(const Eigen::Vector3f& ray_S) const
{
    return underlying()->farDistImpl(ray_S);
}


template<typename DerivedT>
float SensorBase<DerivedT>::measurementFromPoint(const Eigen::Vector3f& point_S) const
{
    return underlying()->measurementFromPointImpl(point_S);
}



template<typename DerivedT>
bool SensorBase<DerivedT>::pointInFrustum(const Eigen::Vector3f& point_S) const
{
    return underlying()->pointInFrustumImpl(point_S);
}



template<typename DerivedT>
bool SensorBase<DerivedT>::pointInFrustumInf(const Eigen::Vector3f& point_S) const
{
    return underlying()->pointInFrustumInfImpl(point_S);
}



template<typename DerivedT>
bool SensorBase<DerivedT>::sphereInFrustum(const Eigen::Vector3f& centre_S,
                                           const float radius) const
{
    return underlying()->sphereInFrustumImpl(centre_S, radius);
}



template<typename DerivedT>
bool SensorBase<DerivedT>::sphereInFrustumInf(const Eigen::Vector3f& centre_S,
                                              const float radius) const
{
    return underlying()->sphereInFrustumInfImpl(centre_S, radius);
}



template<typename DerivedT>
std::string SensorBase<DerivedT>::type()
{
    return DerivedT::typeImpl();
}



template<typename DerivedT>
const DerivedT* SensorBase<DerivedT>::underlying() const
{
    return static_cast<const DerivedT*>(this);
}



template<typename DerivedT>
void se::SensorBase<DerivedT>::Config::readYaml(const std::string& filename)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << filename << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << filename << "\n";
        return;
    }

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default sensor configuration, no \"sensor\" section found in "
                  << filename << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_int(node, "width", width);
    se::yaml::subnode_as_int(node, "height", height);
    se::yaml::subnode_as_float(node, "near_plane", near_plane);
    se::yaml::subnode_as_float(node, "far_plane", far_plane);

    T_BS = Eigen::Isometry3f::Identity();

    if (!node["T_BS"].isNone()) {
        se::yaml::subnode_as_eigen_matrix4f(node, "T_BS", T_BS.matrix());
    }

    if (!node["t_BS"].isNone()) {
        Eigen::Vector3f t_BS;
        se::yaml::subnode_as_eigen_vector3f(node, "t_BS", t_BS);
        T_BS.translation() = t_BS;
    }

    if (!node["R_BS"].isNone()) {
        Eigen::Matrix3f R_BS;
        se::yaml::subnode_as_eigen_matrix3f(node, "R_BS", R_BS);
        T_BS.linear() = R_BS;
    }
}


} // namespace se

#endif // SE_SENSOR_IMPL_HPP
