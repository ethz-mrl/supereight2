/*
 * SPDX-FileCopyrightText: 2025 Mobile Robotics Lab, ETH Zurich, Technical University of Munich
 * SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_READER_KITTI_HPP
#define SE_READER_KITTI_HPP

#include "reader_base.hpp"

namespace se {

/** Reader for the KITTI odometry dataset.
 * https://www.cvlibs.net/datasets/kitti/eval_odometry.php
 */
struct KITTIReader : public Reader {
    KITTIReader(const Config& c);

    /** Restart reading from the beginning of the sequence. */
    void restart() override;

    /** Return the string `"KITTIReader"`. */
    std::string name() const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /** Transformation from the first body frame B0 (the first pose of cam0, the left grayscale
     * camera) to the world frame W. The KITTI odometry dataset ground truth poses are with respect
     * to the first pose of the left camera (cam0). For convenience we define the world frame to be
     * the roughly z-up frame with its origin roughly at ground level below the first pose of the
     * left camera.
     */
    static const Eigen::Isometry3f T_WB0;

    /** Sorted filenames of the LiDAR scans. */
    std::vector<std::string> scan_filenames_;
    /** Sorted filenames of the colour images from cam2. */
    std::vector<std::string> rgb_filenames_;

    /** Read the next colour image into \p colour_image. If no colour images exist return
     * ReaderStatus:error.
     */
    ReaderStatus nextColour(Image<RGB>& colour_image) override;

    /** Read the next LiDAR scan into \p batch. The value of \p batch_interval is ignored. */
    ReaderStatus nextRayBatch(
        const float batch_interval,
        std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                    Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>& batch)
        override;

    /** Read the pose corresponding to \p frame into \p T_WB. Frame B is z-forward, x-right and
     * corresponds to cam0 (left grayscale). Frame W is roughly z-up and has its origin 1.65 m below
     * the first cam0 position in the sequence. See se::KITTIReader::T_WB0 for details. */
    ReaderStatus readPose(Eigen::Isometry3f& T_WB, const size_t frame);
};

} // namespace se

#endif //SE_READER_KITTI_HPP
