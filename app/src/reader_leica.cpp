/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_leica.hpp"

#include <iostream>
#include <se/common/filesystem.hpp>

/** A timestamped ground truth pose (VIO estimate) .
 */
struct LeicaPoseEntry {
    uint64_t timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;

    /** Initialize an invalid LeicaPoseEntry.
     */
    LeicaPoseEntry() = default;

    LeicaPoseEntry(const double t, const Eigen::Vector3f& p, const Eigen::Quaternionf& o) :
            timestamp(t), position(p), orientation(o)
    {
    }

    /**
     * Initialize using a single-line string from a Leica trajectory.csv
     */
    LeicaPoseEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ',', true);
        timestamp = std::stol(columns[1].c_str());
        position =
            Eigen::Vector3f(std::stof(columns[2]), std::stof(columns[3]), std::stof(columns[4]));
        orientation = Eigen::Quaternionf(std::stof(columns[5]),
                                         std::stof(columns[6]),
                                         std::stof(columns[7]),
                                         std::stof(columns[8]));
    }

    /** Return a single-line string representation of the ground truth (VIO) pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + "," + std::to_string(position.x()) + ","
            + std::to_string(position.y()) + "," + std::to_string(position.z()) + ","
            + std::to_string(orientation.x()) + "," + std::to_string(orientation.y()) + ","
            + std::to_string(orientation.z()) + "," + std::to_string(orientation.w());
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** A timestamped ground truth pose (VIO estimate) .
 */
struct LeicaLiDAREntry {
    uint64_t timestamp;
    Eigen::Vector3f position;
    int intensity;

    /** Initialize an invalid LeicaPoseEntry.
     */
    LeicaLiDAREntry() = default;

    LeicaLiDAREntry(const double t, const Eigen::Vector3f& p, const int intensity) :
            timestamp(t), position(p), intensity(intensity)
    {
    }

    /**
     * Initialize using a single-line string from a Leica trajectory.csv
     */
    LeicaLiDAREntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ',', true);
        timestamp = std::stol(columns[0].c_str());
        position =
            Eigen::Vector3f(std::stof(columns[1]), std::stof(columns[2]), std::stof(columns[3]));
        intensity = std::stoi(columns[4]);
    }

    /** Return a single-line string representation of the ground truth (VIO) pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + "," + std::to_string(position.x()) + ","
            + std::to_string(position.y()) + "," + std::to_string(position.z()) + ","
            + std::to_string(intensity);
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Generate a ground truth file from poses and write it in a temporary file.
 */
std::string write_ground_truth_tmp(const std::string& path,
                                   const std::vector<LeicaPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = path + "/trajectory_se.csv";
    std::ofstream fs(tmp_filename, std::ios::out);
    if (!fs.good()) {
        std::cerr << "Error: Could not write trajectory file " << tmp_filename << "\n";
        return "";
    }
    // Write the header
    fs << "# timestamp, tx, ty, tz, qx, qy, qz, qw\n";
    // Write each of the associated poses
    for (size_t i = 0; i < poses.size(); ++i) {
        fs << poses[i].string() << "\n";
    }
    return tmp_filename;
}



se::LeicaReader::LeicaReader(const se::Reader::Config& c) : se::Reader(c)
{
    // Ensure a valid directory was provided
    if (!stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
        return;
    }
    std::cout << "created leica reader" << std::endl;

    // Setup stream of lidar data
    lidar_stream_.open(sequence_path_ + "/lidar.csv");
    if (!lidar_stream_.good()) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ + "/lidar.csv could not be found \n";
        return;
    }
    std::string line;
    std::getline(lidar_stream_, line);

    // initialise current LiDAR timestamp_
    ray_timestamp_ = 0;


    // Setup stream of trajectory data
    trajectory_stream_.open(sequence_path_ + "/trajectory.csv");
    if (!trajectory_stream_.good()) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ + "/trajectory.csv could not be found \n";
        return;
    }
    std::string line2;
    int numberOfLines2 = 0;
    while (std::getline(trajectory_stream_, line2))
        numberOfLines2++;
    if (numberOfLines2 - 1 <= 0) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: No Pose Data present in: " << sequence_path_ + "/trajectory.csv\n";
        return;
    }

    // set reading position to second line
    trajectory_stream_.clear();
    trajectory_stream_.seekg(0, std::ios::beg);
    std::getline(trajectory_stream_, line2);

    // Set reading positions for first measurements (t_ray > t_pose; required for interpolation)
    std::getline(lidar_stream_, line);
    LeicaLiDAREntry lidarMeas(line);

    std::getline(trajectory_stream_, line2);
    LeicaPoseEntry pose(line2);

    while (lidarMeas.timestamp < pose.timestamp) {
        std::getline(lidar_stream_, line);
        lidarMeas = LeicaLiDAREntry(line);
    }
    ray_timestamp_ = lidarMeas.timestamp;

    // Save first previous pose
    ts_prev_ = pose.timestamp;
    pos_prev_ = pose.position;
    ori_prev_ = pose.orientation;

    // Save first current pose
    std::getline(trajectory_stream_, line2);
    pose = LeicaPoseEntry(line2);
    ts_curr_ = pose.timestamp;
    pos_curr_ = pose.position;
    ori_curr_ = pose.orientation;

    while (ts_curr_ < ray_timestamp_) {
        std::getline(trajectory_stream_, line2);
        ts_prev_ = ts_curr_;
        pos_prev_ = pos_curr_;
        ori_prev_ = ori_curr_;
        pose = LeicaPoseEntry(line2);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;
    }
}



void se::LeicaReader::restart()
{
    // ToDo: need to reset Reader Stream here?
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
    }
}



std::string se::LeicaReader::name() const
{
    return std::string("LeicaReader");
}



se::ReaderStatus se::LeicaReader::nextDepth(se::Image<float>& /*depth_image*/)
{
    std::clog << "nextDepth() not supported for LeicaReader\n";
    return se::ReaderStatus::error;
}

se::ReaderStatus se::LeicaReader::nextPose(Eigen::Isometry3f& T_WB)
{
    std::string line;
    LeicaPoseEntry pose;
    T_WB.setIdentity();

    while (ray_timestamp_ > ts_curr_) {
        // Get next {pse
        std::getline(trajectory_stream_, line);
        if (!trajectory_stream_.good())
            return se::ReaderStatus::eof;
        ts_prev_ = ts_curr_;
        pos_prev_ = pos_curr_;
        ori_prev_ = ori_curr_;
        pose = LeicaPoseEntry(line);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;
    }

    // interpolate between poses
    double r =
        (static_cast<double>(ray_timestamp_ - ts_prev_)) / static_cast<double>(ts_curr_ - ts_prev_);
    T_WB.translation() = r * pos_curr_ + (1. - r) * pos_prev_;
    T_WB.linear() = ori_prev_.slerp(r, ori_curr_).toRotationMatrix();

    return se::ReaderStatus::ok;
}

se::ReaderStatus se::LeicaReader::nextRayBatch(
    const float batch_interval,
    std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
        rayPoseBatch)
{
    std::string lidarLine;
    // Get 1 Line of LiDAR measurements = 1 ray
    std::getline(lidar_stream_, lidarLine);
    if (!lidar_stream_.good()) // Reached end of file
        return se::ReaderStatus::eof;

    // Get 1st entry in current interval
    LeicaLiDAREntry ray;
    ray = LeicaLiDAREntry(lidarLine);
    uint64_t t0 = ray.timestamp;
    ray_timestamp_ = ray.timestamp;

    std::string trajectoryLine;
    LeicaPoseEntry pose;
    Eigen::Isometry3f T_WB = Eigen::Isometry3f::Identity();

    while (ray_timestamp_ > ts_curr_) {
        // Get next pose
        std::getline(trajectory_stream_, trajectoryLine);
        if (!trajectory_stream_.good())
            return se::ReaderStatus::eof;
        ts_prev_ = ts_curr_;
        pos_prev_ = pos_curr_;
        ori_prev_ = ori_curr_;
        pose = LeicaPoseEntry(trajectoryLine);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;
    }

    // interpolate between poses
    double r =
        (static_cast<double>(ray_timestamp_ - ts_prev_)) / static_cast<double>(ts_curr_ - ts_prev_);
    T_WB.translation() = r * pos_curr_ + (1. - r) * pos_prev_;
    T_WB.linear() = ori_prev_.slerp(r, ori_curr_).toRotationMatrix();
    rayPoseBatch.push_back(std::pair<Eigen::Isometry3f, Eigen::Vector3f>(T_WB, ray.position));


    // now get interval
    while ((ray_timestamp_ - t0) * 1e-09 <= batch_interval) {
        std::getline(lidar_stream_, lidarLine);
        if (!lidar_stream_.good()) // Reached end of file
            return se::ReaderStatus::eof;

        // Lidar measurement
        LeicaLiDAREntry ray;
        ray = LeicaLiDAREntry(lidarLine);
        ray_timestamp_ = ray.timestamp;

        // Pose interpolated
        T_WB.setIdentity();

        while (ray_timestamp_ > ts_curr_) {
            // Get next pose
            std::getline(trajectory_stream_, trajectoryLine);
            if (!trajectory_stream_.good())
                return se::ReaderStatus::eof;
            ts_prev_ = ts_curr_;
            pos_prev_ = pos_curr_;
            ori_prev_ = ori_curr_;
            pose = LeicaPoseEntry(trajectoryLine);
            ts_curr_ = pose.timestamp;
            pos_curr_ = pose.position;
            ori_curr_ = pose.orientation;
        }

        // interpolate between poses
        double r = (static_cast<double>(ray_timestamp_ - ts_prev_))
            / static_cast<double>(ts_curr_ - ts_prev_);
        T_WB.translation() = r * pos_curr_ + (1. - r) * pos_prev_;
        T_WB.linear() = ori_prev_.slerp(r, ori_curr_).toRotationMatrix();
        rayPoseBatch.push_back(std::pair<Eigen::Isometry3f, Eigen::Vector3f>(T_WB, ray.position));
    }

    return se::ReaderStatus::ok;
}
