/*
 * SPDX-FileCopyrightText: 2025 Mobile Robotics Lab, ETH Zurich, Technical University of Munich
 * SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_kitti.hpp"

#include <algorithm>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <se/common/filesystem.hpp>

namespace se {

struct VelodynePoint {
    float x;
    float y;
    float z;
    float reflectivity;

    std::istream& read(std::istream& stream)
    {
        // Velodyne .bin files store the x, y and z coordinates, followed by the reflectivity as
        // consecutive 32-bit floats in little-endian byte order.
        // XXX: The following only works for little-endian systems but it's unlikely this code will
        // ever run on a system with different endianess.
        stream.read(reinterpret_cast<char*>(this), sizeof(VelodynePoint));
        return stream;
    }
};



static inline bool kitti_is_valid(const std::string& path)
{
    return stdfs::is_directory(path) && stdfs::is_directory(path + "/velodyne")
        && stdfs::is_regular_file(path + "/poses.txt");
}



static inline std::vector<std::string> kitti_find_files(const std::string& directory,
                                                        const std::string& extension)
{
    std::vector<std::string> filenames;
    std::error_code err;
    stdfs::directory_iterator d(directory, stdfs::directory_options::follow_directory_symlink, err);
    if (err) {
        // Error opening the directory.
        return filenames;
    }
    for (const auto& p : d) {
        if (p.path().extension() == extension) {
            filenames.push_back(p.path());
        }
    }
    std::sort(filenames.begin(), filenames.end());
    return filenames;
}



const Eigen::Isometry3f KITTIReader::T_WB0 = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX())
    * Eigen::Translation3f(0.0f, 0.0f, 1.65f);



KITTIReader::KITTIReader(const Reader::Config& c) : Reader(c)
{
    if (!kitti_is_valid(sequence_path_)) {
        status_ = ReaderStatus::error;
        std::cerr << "Error: The KITTI sequence path must be a directory that contains"
                     " a velodyne subdirectory and a poses.txt file\n";
        return;
    }
    // Disable exceptions from the ground truth file stream.
    ground_truth_fs_.exceptions(std::ios_base::goodbit);
    scan_filenames_ = kitti_find_files(sequence_path_ + "/velodyne", ".bin");
    rgb_filenames_ = kitti_find_files(sequence_path_ + "/image_2", ".png");
    num_frames_ = scan_filenames_.size();
    if (!rgb_filenames_.empty()) {
        cv::Mat image_data = cv::imread(rgb_filenames_.front(), cv::IMREAD_COLOR);
        if (image_data.empty()) {
            std::cerr << "Error: Could not read RGB image " << rgb_filenames_.front() << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
        colour_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
        has_colour_ = true;
    }
}



void KITTIReader::restart()
{
    Reader::restart();
    if (kitti_is_valid(sequence_path_)) {
        status_ = ReaderStatus::ok;
    }
    else {
        status_ = ReaderStatus::error;
    }
}



std::string KITTIReader::name() const
{
    return std::string("KITTIReader");
}



ReaderStatus KITTIReader::nextColour(Image<RGB>& colour_image)
{
    if (rgb_filenames_.empty()) {
        return ReaderStatus::error;
    }

    cv::Mat image_data = cv::imread(rgb_filenames_[frame_], cv::IMREAD_COLOR);
    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

    cv::Mat colour_data;
    cv::cvtColor(image_data, colour_data, cv::COLOR_BGR2RGB);

    // Resize the output image if needed.
    if ((colour_image.width() != colour_data.cols) || (colour_image.height() != colour_data.rows)) {
        colour_image = se::Image<RGB>(colour_data.cols, colour_data.rows);
    }

    cv::Mat wrapper(colour_image.height(), colour_image.width(), CV_8UC3, colour_image.data());
    colour_data.copyTo(wrapper);

    return se::ReaderStatus::ok;
}



ReaderStatus KITTIReader::nextRayBatch(
    const float /* batch_interval */,
    std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>& batch)
{
    batch.clear();
    // Read the pose first since we don't currently support ray batches without a ground truth pose.
    Eigen::Isometry3f T_WB;
    const ReaderStatus status = readPose(T_WB, frame_);
    if (status != ReaderStatus::ok) {
        return status;
    }

    std::ifstream f(scan_filenames_[frame_]);
    f.exceptions(std::ios_base::goodbit); // Ignore exceptions while reading.
    while (f.good()) {
        VelodynePoint point;
        if (point.read(f)) {
            batch.emplace_back(T_WB, Eigen::Vector3f(point.x, point.y, point.z));
        }
    }
    if (verbose_ >= 2) {
        std::clog << "nextRayBatch(): read " << batch.size() << " points from "
                  << scan_filenames_[frame_] << "\n";
    }
    if (f.eof()) {
        return ReaderStatus::ok;
    }
    return ReaderStatus::error;
}



ReaderStatus KITTIReader::readPose(Eigen::Isometry3f& T_WB, const size_t frame)
{
    // The KITTI poses.txt contains transformations from the current coordinate frame of cam0 (B) to
    // the initial coordinate frame of cam0 (B0).
    Eigen::Isometry3f T_B0B = Eigen::Isometry3f::Identity();
    while (true) {
        ground_truth_fs_ >> T_B0B(0, 0);
        ground_truth_fs_ >> T_B0B(0, 1);
        ground_truth_fs_ >> T_B0B(0, 2);
        ground_truth_fs_ >> T_B0B(0, 3);
        ground_truth_fs_ >> T_B0B(1, 0);
        ground_truth_fs_ >> T_B0B(1, 1);
        ground_truth_fs_ >> T_B0B(1, 2);
        ground_truth_fs_ >> T_B0B(1, 3);
        ground_truth_fs_ >> T_B0B(2, 0);
        ground_truth_fs_ >> T_B0B(2, 1);
        ground_truth_fs_ >> T_B0B(2, 2);
        ground_truth_fs_ >> T_B0B(2, 3);
        if (ground_truth_fs_.bad() || ground_truth_fs_.fail()) {
            return ReaderStatus::error;
        }
        else if (ground_truth_fs_.eof()) {
            return ReaderStatus::eof;
        }
        // Skip ground truth data until the ones corresponding to the current frame are found. This
        // only happens when frames are dropped.
        ground_truth_frame_++;
        if (ground_truth_frame_ < frame) {
            continue;
        }
        T_WB = T_WB0 * T_B0B;
        return ReaderStatus::ok;
    }
}

} // namespace se
