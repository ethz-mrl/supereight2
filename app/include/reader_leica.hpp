/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __READER_LEICA_HPP
#define __READER_LEICA_HPP

#include "reader_base.hpp"

namespace se {

/** Reader for Leica style datasets.
 * Lidar csv ts | x | y | z | intensity
 */
class LeicaReader : public Reader {
    public:
    /** Construct a LeicaReader from a Config.
     *
     * \param[in] c The configuration struct to use.
     */
    LeicaReader(const Config& c);

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"LeicaReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /// LidarStream
    std::ifstream lidar_stream_;
    uint64_t ray_timestamp_ = 0;
    /// TrajectoryStream
    std::ifstream trajectory_stream_;
    uint64_t ts_prev_ = 0;
    uint64_t ts_curr_ = 0;
    Eigen::Vector3f pos_prev_, pos_curr_;
    Eigen::Quaternionf ori_prev_, ori_curr_;

    float azimuth_angular_resolution_;
    float elevation_angular_resolution_;


    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextPose(Eigen::Isometry3f& T_WB);

    ReaderStatus nextRayBatch(
        const float batch_interval,
        std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                    Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
            rayPoseBatch);

    /** Return the filenames of LIDAR scans in PCD formatin the supplied directory.
     * LIDAR scans are considered those whose name conforms to the pattern
     * cloud_XXXXXXXXXX_XXXXXXXXX.pcd where X is a digit 0-9.
     *
     * \param[in] dir The directory inside which to look for PCD files.
     * \return The filenames of the PCD files found in lexicographical order.
     */
    static std::string getScanFilename(const std::string& dir);
};

} // namespace se

#endif //__READER_LEICA_HPP
