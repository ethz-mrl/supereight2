/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_interiornet.hpp"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <se/common/filesystem.hpp>

struct InteriorNetIntrinsics {
    static constexpr float f_x = 600;
    static constexpr float f_y = 600;
    static constexpr float c_x = 320;
    static constexpr float c_y = 240;
};

/** The filename of a depth or RGB image and its timestamp.
 */
struct InteriorNetImageEntry {
    double timestamp;
    std::string filename;

    /** Initialize an invalid InteriorNetImageEntry.
     */
    InteriorNetImageEntry() : timestamp(NAN)
    {
    }

    /** Initialize using a single-line string from a InteriorNet data.csv.
     * \warning No error checking is performed in this function, it should be
     * performed by the caller.
     */
    InteriorNetImageEntry(const std::string& s)
    {
        std::vector<std::string> columns = se::str_utils::split_str(s, ' ', true);
        timestamp = std::stod(columns[0]);
        if (!columns[1].empty() && columns[1][columns[1].size() - 1] == '\r') {
            columns[1].erase(columns[1].size() - 1);
        }
        filename = columns[1];
    }
};



/** A timestamped ground truth pose and its associated depth and RGB images.
 */
struct InteriorNetPoseEntry {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::string depth_filename;
    std::string rgb_filename;

    /** Initialize an invalid InteriorNetPoseEntry.
     */
    InteriorNetPoseEntry()
    {
    }

    InteriorNetPoseEntry(const double t,
                         const Eigen::Vector3f& p,
                         const Eigen::Quaternionf& o,
                         const std::string& df,
                         const std::string& rf) :
            timestamp(t), position(p), orientation(o), depth_filename(df), rgb_filename(rf)
    {
    }

    /** Initialize using a single-line string from a InteriorNet groundtruth.txt.
     * depth_filename and rgb_filename will not be initialized.
     * \warning No error checking is performed in this function, it should be
     * performed by the caller.
     */
    InteriorNetPoseEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ' ', true);
        position =
            Eigen::Vector3f(std::stof(columns[10]), std::stof(columns[11]), std::stof(columns[12]));
        Eigen::Quaternionf q_WB = Eigen::Quaternionf(std::stof(columns[6]),
                                                     std::stof(columns[7]),
                                                     std::stof(columns[8]),
                                                     std::stof(columns[9]));
        Eigen::Quaternionf q_BS = Eigen::Quaternionf(0, 1, 0, 0);
        orientation = q_WB * q_BS;
    }

    /** Return a single-line string representation of the ground truth pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + " " + rgb_filename + " " + depth_filename
            + " " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " "
            + std::to_string(position.z()) + " " + std::to_string(orientation.x()) + " "
            + std::to_string(orientation.y()) + " " + std::to_string(orientation.z()) + " "
            + std::to_string(orientation.w());
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/** Read a InteriorNet data.csv into an std::vector of InteriorNetImageEntry.
 * Return an empty std::vector if the file was not in the correct format.
 */
std::vector<InteriorNetImageEntry> read_interiornet_image_list(const std::string& filename)
{
    std::vector<InteriorNetImageEntry> images;
    std::ifstream fs(filename, std::ios::in);
    if (!fs.good()) {
        std::cerr << "Error: Could not read file " << filename << "\n";
        return images;
    }
    // Read all data lines
    for (std::string line; std::getline(fs, line);) {
        // Ignore comment lines
        if (line[0] == '#') {
            continue;
        }
        // Data line read, split on commas
        std::replace(line.begin(), line.end(), ',', ' ');
        const std::vector<std::string> columns = se::str_utils::split_str(line, ' ', true);
        // Ensure it has the expected number of columns
        if (columns.size() != 2) {
            std::cerr << "Error: Invalid format in data line " << images.size() + 1 << " of "
                      << filename << "\n";
            std::cerr << "Error: The format of each line in an InteriorNet data.csv file must be: "
                      << "timestamp,filename\n";
            images.clear();
            return images;
        }
        // Ensure that the timestamp is a positive float
        if (!se::str_utils::is_float(columns[0], false)) {
            std::cerr << "Error: Invalid timestamp in data line " << images.size() + 1 << " of "
                      << filename << "\n";
            std::cerr << "Error: The InteriorNet timestamps must be non-negative floats\n";
            images.clear();
            return images;
        }
        // Add the image info
        images.emplace_back(line);
    }
    if (images.empty()) {
        std::cerr << "Error: Empty file " << filename << "\n";
    }
    return images;
}



/** Read a InteriorNet groundtruth.txt into an std::vector of InteriorNetPoseEntry.
 * Return an empty std::vector if the file was not in the correct format.
 */
std::vector<InteriorNetPoseEntry> read_interiornet_ground_truth(const std::string& filename)
{
    std::vector<InteriorNetPoseEntry> poses;
    std::ifstream fs(filename, std::ios::in);
    if (!fs.good()) {
        std::cerr << "Error: Could not read ground truth file " << filename << "\n";
        return poses;
    }
    // Read all data lines
    for (std::string line; std::getline(fs, line);) {
        // Ignore comment lines
        if (line[0] == '#') {
            continue;
        }
        // Data line read, split on spaces
        const std::vector<std::string> columns = se::str_utils::split_str(line, ' ', true);
        // Ensure it has the expected number of columns
        if (columns.size() != 15) {
            continue;
        }
        // Ensure that the timestamp is a positive float
        if (!se::str_utils::is_float(columns[0], false)) {
            std::cerr << "Error: Invalid timestamp in data line " << poses.size() + 1 << " of "
                      << filename << "\n";
            std::cerr
                << "Error: The InteriorNet ground truth timestamps must be non-negative floats\n";
            poses.clear();
            return poses;
        }
        // Ensure that the rest of the data are floats
        for (size_t i = 0; i < columns.size(); ++i) {
            if (!se::str_utils::is_float(columns[i])) {
                std::cerr << "Error: Invalid pose in data line " << poses.size() + 1 << " of "
                          << filename << "\n";
                std::cerr << "Error: The InteriorNet ground truth pose data must be floats\n";
                poses.clear();
                return poses;
            }
        }
        // Add the pose
        poses.emplace_back(line);
    }
    if (poses.empty()) {
        std::cerr << "Error: Empty ground truth file " << filename << "\n";
    }
    return poses;
}



/** Generate a ground truth file from poses and write it in a temporary file.
 */
std::string write_ground_truth_tmp(const std::vector<InteriorNetPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = stdfs::temp_directory_path() / "tum_gt.txt";
    std::ofstream fs(tmp_filename, std::ios::out);
    if (!fs.good()) {
        std::cerr << "Error: Could not write associated ground truth file " << tmp_filename << "\n";
        return "";
    }
    // Write the header
    fs << "# Association of rgb images, depth images and ground truth poses\n";
    fs << "# ID timestamp rgb_filename depth_filename tx ty tz qx qy qz qw\n";
    // Write each of the associated poses
    for (size_t i = 0; i < poses.size(); ++i) {
        fs << std::setw(6) << std::setfill('0') << i << " " << poses[i].string() << "\n";
    }
    return tmp_filename;
}



void association(std::vector<InteriorNetPoseEntry>& gt_poses,
                 const std::vector<InteriorNetImageEntry>& depth_images,
                 const std::vector<InteriorNetImageEntry>& rgb_images)
{
    for (unsigned int i = 0; i < gt_poses.size(); i++) {
        gt_poses[i].timestamp = depth_images[i].timestamp;
        gt_poses[i].depth_filename = depth_images[i].filename;
        gt_poses[i].rgb_filename = rgb_images[i].filename;
    }
}



// InteriorNetReader implementation
constexpr float se::InteriorNetReader::interiornet_inverse_scale_;



se::InteriorNetReader::InteriorNetReader(const se::Reader::Config& c) :
        se::Reader(c),
        inverse_scale_(c.inverse_scale == 0.0f ? interiornet_inverse_scale_ : c.inverse_scale)
{
    // Ensure sequence_path_ refers to a valid InteriorNet directory structure.
    if (!stdfs::is_directory(sequence_path_)
        || !stdfs::is_directory(sequence_path_ + "/depth0/data")
        || !stdfs::is_directory(sequence_path_ + "/cam0/data")
        || !stdfs::is_regular_file(sequence_path_ + "/depth0/data.csv")
        || !stdfs::is_regular_file(sequence_path_ + "/cam0/data.csv")) {
        status_ = se::ReaderStatus::error;
        std::cerr
            << "Error: The InteriorNet sequence path must be a directory that contains"
            << " depth0/data and cam0/data subdirectories and depth0/data.csv and cam0/data.csv files\n";
        return;
    }
    // Read the image information from the data.csv.
    std::vector<InteriorNetImageEntry> depth_images =
        read_interiornet_image_list(sequence_path_ + "/depth0/data.csv");
    std::vector<InteriorNetImageEntry> rgb_images =
        read_interiornet_image_list(sequence_path_ + "/cam0/data.csv");
    // Read and associate the ground truth file if needed
    if (!ground_truth_file_.empty()) {
        // Read the the ground truth poses and timestamps
        std::vector<InteriorNetPoseEntry> gt_poses =
            read_interiornet_ground_truth(ground_truth_file_);
        if (gt_poses.empty()) {
            status_ = se::ReaderStatus::error;
            return;
        }
        association(gt_poses, depth_images, rgb_images);

        // Generate the associated ground truth file
        const std::string generated_filename = write_ground_truth_tmp(gt_poses);
        // Close the original ground truth file and open the generated one
        ground_truth_fs_.close();
        ground_truth_fs_.open(generated_filename, std::ios::in);
        if (!ground_truth_fs_.good()) {
            std::cerr << "Error: Could not read generated ground truth file " << generated_filename
                      << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }
    // Get the filenames of the depth images.
    depth_filenames_.resize(depth_images.size());
    std::transform(depth_images.begin(),
                   depth_images.end(),
                   depth_filenames_.begin(),
                   [](const auto& e) { return e.filename; });
    // Get the filenames of the RGB images.
    rgb_filenames_.resize(rgb_images.size());
    std::transform(rgb_images.begin(), rgb_images.end(), rgb_filenames_.begin(), [](const auto& e) {
        return e.filename;
    });
    // Get the total number of frames.
    num_frames_ = depth_filenames_.size();
    // Set the depth image resolution to that of the first depth image.
    if (!depth_filenames_.empty()) {
        const std::string first_depth_filename =
            sequence_path_ + "/depth0/data/" + depth_filenames_[0];
        cv::Mat image_data = cv::imread(first_depth_filename.c_str(), cv::IMREAD_UNCHANGED);

        if (image_data.data == NULL) {
            std::cerr << "Error: Could not read depth image " << first_depth_filename << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }

        depth_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
    }

    projection_inv_ = cv::Mat(depth_image_res_.y(), depth_image_res_.x(), CV_32FC1);

    for (int y = 0; y < depth_image_res_.y(); y++) {
        for (int x = 0; x < depth_image_res_.x(); x++) {
            projection_inv_.at<float>(y, x) = 1
                / sqrt((1
                        + (x - InteriorNetIntrinsics::c_x) * (x - InteriorNetIntrinsics::c_x)
                            / (InteriorNetIntrinsics::f_x * InteriorNetIntrinsics::f_x)
                        + (y - InteriorNetIntrinsics::c_y) * (y - InteriorNetIntrinsics::c_y)
                            / (InteriorNetIntrinsics::f_y * InteriorNetIntrinsics::f_y)));
        }
    }

    // Set the colour image resolution to that of the first colour image.
    if (!rgb_filenames_.empty()) {
        const std::string first_rgb_filename = sequence_path_ + "/cam0/data/" + rgb_filenames_[0];
        cv::Mat image_data = cv::imread(first_rgb_filename.c_str(), cv::IMREAD_COLOR);

        if (image_data.data == NULL) {
            std::cerr << "Error: Could not read RGB image " << first_rgb_filename << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
        colour_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
    }
    has_colour_ = true;
}



void se::InteriorNetReader::restart()
{
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
    }
}



std::string se::InteriorNetReader::name() const
{
    return std::string("InteriorNetReader");
}



se::ReaderStatus se::InteriorNetReader::nextDepth(se::Image<float>& depth_image)
{
    if (frame_ >= num_frames_) {
        return se::ReaderStatus::error;
    }
    const std::string filename = sequence_path_ + "/depth0/data/" + depth_filenames_[frame_];
    // Read the image data.

    // Read the image data.
    cv::Mat image_data = cv::imread(filename.c_str(), cv::IMREAD_UNCHANGED);
    cv::Mat depth_data;
    image_data.convertTo(depth_data, CV_32FC1, inverse_scale_);
    depth_data = depth_data.mul(projection_inv_);

    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

    assert(depth_image_res_.x() == static_cast<int>(image_data.cols));
    assert(depth_image_res_.y() == static_cast<int>(image_data.rows));
    // Resize the output image if needed.
    if ((depth_image.width() != depth_image_res_.x())
        || (depth_image.height() != depth_image_res_.y())) {
        depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
    }

    cv::Mat wrapper_mat(depth_data.rows, depth_data.cols, CV_32FC1, depth_image.data());
    depth_data.copyTo(wrapper_mat);
    return se::ReaderStatus::ok;
}



se::ReaderStatus se::InteriorNetReader::nextColour(se::Image<RGB>& colour_image)
{
    if (frame_ >= num_frames_) {
        return se::ReaderStatus::error;
    }
    const std::string filename = sequence_path_ + "/cam0/data/" + rgb_filenames_[frame_];

    cv::Mat image_data = cv::imread(filename.c_str(), cv::IMREAD_COLOR);

    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

    cv::Mat colour_data;
    cv::cvtColor(image_data, colour_data, cv::COLOR_BGR2RGB);

    assert(colour_image_res_.x() == static_cast<int>(colour_data.cols));
    assert(colour_image_res_.y() == static_cast<int>(colour_data.rows));
    // Resize the output image if needed.
    if ((colour_image.width() != colour_image_res_.x())
        || (colour_image.height() != colour_image_res_.y())) {
        colour_image = se::Image<RGB>(colour_image_res_.x(), colour_image_res_.y());
    }

    cv::Mat wrapper_mat(colour_data.rows, colour_data.cols, CV_8UC3, colour_image.data());
    colour_data.copyTo(wrapper_mat);
    return se::ReaderStatus::ok;
}
