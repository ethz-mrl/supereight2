#include <iostream>
#include <string>
#include <fstream>
#include <deque>
#include <chrono>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Eigen>

#include <torch/torch.h>
#include <torch/script.h>

#include <se/supereight.hpp>
#include "../include/config.hpp"
#include "se/common/filesystem.hpp"

typedef se::Image<float> DepthFrame;


/// Convert a grayscale OpenCV image to a torch (byte) tensor.
/**
 * \param img   OpenCV image as cv::Mat.
 * \return      Corresponding torch (byte) tensor.
 */
torch::Tensor cvMatToTensor(const cv::Mat& img) {
  return torch::from_blob(img.data, {img.rows, img.cols}, at::kByte);
}

/// Convert a torch (byte) tensor to a grayscale OpenCV image.
/**
 * \param tensor  Torch tensor to convert.
 * \return        Corresponding OpenCV image as cv::Mat.
 */
cv::Mat tensorToCvMatByte(const torch::Tensor& tensor) {
  int width = tensor.sizes()[0];
  int height = tensor.sizes()[1];
  cv::Mat output_mat(cv::Size{height, width}, CV_8UC1, tensor.data_ptr<uchar>());  
  return output_mat.clone();
}

cv::Mat tensorToDepthVis(const torch::Tensor& tensor) {
  torch::Tensor visTensor = ((tensor/10.0).clamp(0.0, 1.0f) * 255.0f).to(torch::kU8);
  cv::Mat visMat = tensorToCvMatByte(visTensor.detach().cpu());
  cv::Mat visRgb;
  cv::applyColorMap(visMat, visRgb, cv::COLORMAP_INFERNO);

  return visRgb.clone();
}

cv::Mat tensorToSigmaVis(const torch::Tensor& tensor) {
  torch::Tensor visTensor = ((tensor/1.0).clamp(0.0, 1.0f) * 255.0f).to(torch::kU8);
  cv::Mat visMat = tensorToCvMatByte(visTensor.detach().cpu());
  cv::Mat visRgb;
  cv::applyColorMap(visMat, visRgb, cv::COLORMAP_JET);

  return visRgb.clone();
}

cv::Mat tensorToCvMatFloat(const torch::Tensor& tensor) {
  int width = tensor.sizes()[0];
  int height = tensor.sizes()[1];
  cv::Mat output_mat(cv::Size{height, width}, CV_32F, tensor.data_ptr<float>());  
  return output_mat.clone();
}

DepthFrame depthMat2Image(const cv::Mat &inputDepth) {

    // Initialise and copy
    if(inputDepth.type() != CV_32FC1) {
    throw std::runtime_error("Only implemented for CV_32FC1 cv::Mat");
    }
    DepthFrame output(inputDepth.cols, inputDepth.rows);

    // cv::MAT and DepthFrame keep data stored in row major format.
    if(!inputDepth.isContinuous()) {
    //TODO write down row by row, first iterate rows the columns and add them to a vector
    throw std::runtime_error("Only implemented for continuous cv::Mat");
    }

    memcpy(output.data(), inputDepth.data,
        inputDepth.cols * inputDepth.rows * sizeof(float));

    return output;
}

struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cv::Mat descriptor; ///< The descriptor.
    Eigen::Vector3f point; ///< The 3d point in World coordinates.
    uint64_t landmarkId;
  };

typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>> LandmarkVec;

std::map<uint64_t, LandmarkVec> loadMap(std::string path) {
  std::map<uint64_t, LandmarkVec> landmarks_;
  std::map<uint64_t, std::set<uint64_t>> covisibilities_;
  std::ifstream mapfile(path);

  // read each line
  std::string line;
  std::set<uint64_t> lmIds;
  uint64_t poseId = 0;
  LandmarkVec landmarks;
  while (std::getline(mapfile, line)) {

    // Convert to stringstream
    std::stringstream ss(line);
    
    if(0==line.compare(0, 7,"frame: ")) {
      // store previous set into map
      landmarks_[poseId] = landmarks;
      // get pose id:
      std::stringstream frameSs(line.substr(7,line.size()-1));
      frameSs >> poseId;
      if(!frameSs.eof()) {
        std::string covisStr;
        frameSs >> covisStr; // comma
        frameSs >> covisStr;
        if(0==covisStr.compare("covisibilities:")) {
          while(!frameSs.eof()) {
            uint64_t covisId;
            frameSs >> covisId;
            covisibilities_[poseId].insert(covisId);
          }
        }
      }
      // move to filling next set of landmarks
      landmarks.clear();
    } else {
      if(poseId>0) {
        Landmark landmark;
      
        // get keypoint idx
        size_t keypointIdx;
        std::string keypointIdxString;
        std::getline(ss, keypointIdxString, ',');
        std::stringstream(keypointIdxString) >> keypointIdx;
        
        // get landmark id
        uint64_t landmarkId;
        std::string landmarkIdString;
        std::getline(ss, landmarkIdString, ',');
        std::stringstream(landmarkIdString) >> landmarkId;
        landmark.landmarkId = landmarkId;
        
        // read 3d position
        for(int i=0; i<3; ++i) {
          std::string coordString;
          std::getline(ss, coordString, ',');
          float coord;
          std::stringstream(coordString) >> coord;
          landmark.point[i] = coord;
        }

        // Get descriptor
        std::string descriptorstring;
        std::getline(ss, descriptorstring);
        landmark.descriptor = cv::Mat(1,48,CV_8UC1);
        for(int col=0; col<48; ++col) {
          uint32_t byte;
          std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
          landmark.descriptor.at<uchar>(0,col) = byte;
        }
        lmIds.insert(landmarkId);
        landmarks.push_back(landmark);
      }      
    } 
  }
  if(poseId>0) {
    // store into map
    landmarks_[poseId] = landmarks;
  }
  std::cout << "loaded " << lmIds.size() << " landmarks from " << landmarks_.size() << " poses." << std::endl;
  return landmarks_;
}


int main(int argc, char **argv) {

    // argv [1] --> Supereight config file
    // argv [2] --> dataset path
    // argv [3] --> mesh output path
    std::string config_filename(argv[1]);
    std::string basePath(argv[2]);
    std::string outputPath(argv[3]);

    // Load network and set pose, landmarks paths
    // TODO: remove hard-coded directories.
    torch::jit::script::Module mvsModel, stereoModel;
    std::string mvsFile = "/usr/wiss/juja/storage/www/group/srl-models/mvs/mvs-sigma.pt";
    std::string stereoFile = "/usr/wiss/juja/storage/www/group/srl-models/unimatch/stereo-indoor-sigma.pt";
    std::string posePath = basePath + "results/okvis2-mvs/okvis2-slam-final-ba_trajectory.csv";
    std::string mapPath = basePath + "results/okvis2-mvs/okvis2-slam-final_map.csv";

    mvsModel = torch::jit::load(mvsFile);
    stereoModel = torch::jit::load(stereoFile, torch::kCUDA);

    // Init SE2 map
    const se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera> config(config_filename);
    std::cout << config;

    // Parameters
    float stereoBaseline = 0.112931;
    float f_orig = config.sensor.fx; // rectified to 512 x 384
    float cu_orig = config.sensor.cx;
    float cv_orig = config.sensor.cy;
    float f_half = 0.5*f_orig;
    float cu_half = 0.5*cu_orig;
    float cv_half = 0.5*cv_orig;
    int src_num = 7;
    int view_num = 8;
    int height = config.sensor.height;
    int width = config.sensor.width;
    Eigen::Matrix4f T_SC = config.sensor.T_BS.matrix(); // T^{IMU}_{Rectified left camera}
    Eigen::Matrix4f T_CS = T_SC.inverse();
    cv::Size orig_size(width, height);
    cv::Size down_size(static_cast<int>(0.5*width), static_cast<int>(0.5*height));
    int downsample_rate = 2;
    float fb = config.sensor.fx * stereoBaseline;

    auto options = torch::TensorOptions()
        .dtype(torch::kFloat32)
        .layout(torch::kStrided)
        .device(torch::kCPU)
        .requires_grad(false);

    auto options_char = torch::TensorOptions()
        .dtype(torch::kU8)
        .layout(torch::kStrided)
        .device(torch::kCPU)
        .requires_grad(false);

    // Camera parameters
    float cur_intr[16] = {f_half, 0.0, cu_half, 0.0,
                          0.0, f_half, cv_half, 0.0,
                          0.0, 0.0, 1.0, 0.0, 
                          0.0, 0.0, 0.0, 1.0};
    float src_intr_i[16] = {0.5f*f_half, 0.0, 0.5f*cu_half, 0.0, 
                            0.0, 0.5f*f_half, 0.5f*cv_half, 0.0, 
                            0.0, 0.0, 1.0, 0.0, 
                            0.0, 0.0, 0.0, 1.0};    
    float cur_inv_intr[16] = {1.0f/src_intr_i[0], 0.0, -src_intr_i[2]/src_intr_i[0], 0.0, 
                              0.0, 1.0f/src_intr_i[5], -src_intr_i[6]/src_intr_i[5], 0.0, 
                              0.0, 0.0, 1.0, 0.0, 
                              0.0, 0.0, 0.0, 1.0};

    auto cur_K = torch::empty({4, 4}, options);
    auto cur_K_a = cur_K.accessor<float, 2>();
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            cur_K_a[i][j] = cur_intr[4 * i + j];
        }
    }

    auto cur_invK = torch::empty({4, 4}, options);
    auto cur_invK_a = cur_invK.accessor<float, 2>();
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            cur_invK_a[i][j] = cur_inv_intr[4 * i + j];
        }
    }
      
    auto src_K = torch::empty({src_num, 4, 4}, options);
    auto src_K_a = src_K.accessor<float, 3>();
    for (size_t view = 0; view < src_num; view ++) {
        for (size_t i =0; i < 4; i ++) {
            for (size_t j = 0; j < 4; j ++) {
                src_K_a[view][i][j] = src_intr_i[4 * i + j];
            }
        }
    }

    // Read camera pose from a file and sparse landmarks
    std::ifstream poseFile(posePath);
    std::map<uint64_t, LandmarkVec> landmarks = loadMap(mapPath);
    std::string line;
    std::deque<std::string> windowLine;
    std::getline(poseFile, line); // skip the first description line
    int frameIdx = 0;

    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);
    se::MapIntegrator integrator(map);
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);
    uint32_t integration_counter = 0;
    cv::Mat mvsfuse_vis = cv::Mat::zeros(2*height, 2*width, CV_8UC3);

    // Main loop
    while (std::getline(poseFile, line)) {
        // Current (ith) state
        std::stringstream sline_k(line);
        std::string gs_k;

        std::vector<float> pq_WSk;
        std::string timestamp_k;
        for (size_t j = 0; j < 8; j++) {
            std::getline(sline_k, gs_k, ',');
            if (j == 0) {
                timestamp_k = gs_k;
            }
            else if (j > 0) {
                pq_WSk.push_back(std::stof(gs_k));
            }
        }

        // {S} to {C}
        Eigen::Matrix4f T_WSk = Eigen::Matrix4f::Identity();
        T_WSk(0,3) = pq_WSk[0]; // x-position
        T_WSk(1,3) = pq_WSk[1]; // y-position
        T_WSk(2,3) = pq_WSk[2]; // z-position

        Eigen::Quaternionf q_WSk;
        q_WSk.w() = pq_WSk[6];
        q_WSk.vec() = Eigen::Vector3f(pq_WSk[3],pq_WSk[4],pq_WSk[5]);
        T_WSk.block<3,3>(0,0) = q_WSk.toRotationMatrix();
        Eigen::Matrix4f T_WCk = T_WSk * T_SC;
        Eigen::Matrix4f T_CkW = T_WCk.inverse();

        // If keyframe, save them in the MVS frames
        bool isKeyframe = false;
        if (landmarks.find(frameIdx) != landmarks.end()) {
            isKeyframe = true;
            std::cout << "Keyframe" << std::endl;
            windowLine.push_back(line);
        }

        // Stereo Network
        auto startStereo = std::chrono::high_resolution_clock::now();
        std::string leftFile_i = basePath + "cam0/rectified/" + timestamp_k + ".png";
        std::string rightFile_i = basePath + "cam1/rectified/" + timestamp_k + ".png";
        cv::Mat left_i = cv::imread(leftFile_i, cv::IMREAD_GRAYSCALE);
        cv::Mat right_i = cv::imread(rightFile_i, cv::IMREAD_GRAYSCALE);
        auto leftTensor = cvMatToTensor(left_i).to(torch::kCUDA);
        auto rightTensor = cvMatToTensor(right_i).to(torch::kCUDA);

        torch::Tensor outputStereo = stereoModel.forward({leftTensor, rightTensor}).toTensor();
        torch::Tensor stereoDisparityT = outputStereo.index({0,torch::indexing::Slice(),torch::indexing::Slice()}).squeeze();
        torch::Tensor stereoDisparitySigmaT = 2.0*outputStereo.index({1,torch::indexing::Slice(),torch::indexing::Slice()}).squeeze();
        torch::Tensor stereoDepthT = fb / stereoDisparityT;
        torch::Tensor stereoSigmaT = (stereoDepthT / stereoDisparityT) * stereoDisparitySigmaT;
        stereoSigmaT.index_put_({torch::isinf(stereoSigmaT)}, 100.0f);
        stereoSigmaT.index_put_({torch::isnan(stereoSigmaT)}, 100.0f);
        cv::Mat stereoDepth = tensorToCvMatFloat(stereoDepthT.to(torch::kFloat).detach().cpu());
        cv::Mat stereoSigma = tensorToCvMatFloat(stereoSigmaT.to(torch::kFloat).detach().cpu());
        auto stopStereo = std::chrono::high_resolution_clock::now();

        // After reaching maximum window and MVS is available
        bool isMVS = false;
        torch::Tensor mvsDepthT, mvsSigmaT;
        cv::Mat cur_sparse_depth, cur_img, sparseDepthVis;
        std::vector<cv::Mat> src_img;
        int numKeypoints = 0;
        auto startMVS = std::chrono::high_resolution_clock::now();
        if (windowLine.size() >= view_num && isKeyframe) {

            numKeypoints = 0;

            // For each frame in a window
            src_img.clear();
            float cur_c2w[16];
            float src_c2ws[7][16];
            int cnt_src = 0;
            for (size_t i = 0; i < view_num; i++) {
                // Read pose
                std::stringstream sline_i(windowLine[i]);
                std::string gs;

                std::vector<float> pq_WSi;
                std::string timestamp_i;
                for (size_t j = 0; j < 8; j++) {
                    std::getline(sline_i, gs, ',');
                    if (j == 0) {
                        timestamp_i = gs;
                    }
                    else if (j > 0) {
                        pq_WSi.push_back(std::stof(gs));
                    }
                }

                // {S} to {C}
                Eigen::Matrix4f T_WSi = Eigen::Matrix4f::Identity();
                T_WSi(0,3) = pq_WSi[0]; // x-position
                T_WSi(1,3) = pq_WSi[1]; // y-position
                T_WSi(2,3) = pq_WSi[2]; // z-position

                Eigen::Quaternionf q_WSi;
                q_WSi.w() = pq_WSi[6];
                q_WSi.vec() = Eigen::Vector3f(pq_WSi[3],pq_WSi[4],pq_WSi[5]);
                T_WSi.block<3,3>(0,0) = q_WSi.toRotationMatrix();
                Eigen::Matrix4f T_WCi = T_WSi * T_SC;
                Eigen::Matrix4f T_CiW = T_WCi.inverse();

                std::string imageFile_i = basePath + "cam0/rectified/" + timestamp_i + ".png";

                if (i != view_num - 1) { // This is source images
                    src_img.push_back(cv::imread(imageFile_i, cv::IMREAD_GRAYSCALE));
                    int cnt_array = 0;
                    for (size_t r = 0; r < 4; r++) {
                        for (size_t c = 0; c < 4; c++) {
                            src_c2ws[cnt_src][cnt_array] = T_WCi(r,c);
                            cnt_array ++;
                        }
                    }
                    cnt_src ++;
                }
                else { // This is the current image
                    cur_img = cv::imread(imageFile_i, cv::IMREAD_GRAYSCALE);
                    cv::cvtColor(cur_img, sparseDepthVis, cv::COLOR_GRAY2BGR);
                    int cnt_array = 0;
                    for (size_t r = 0; r < 4; r++) {
                        for (size_t c = 0; c < 4; c++) {
                            cur_c2w[cnt_array] = T_WCi(r,c);
                            cnt_array ++;
                        }
                    }

                    // Simulate sparse depth image
                    cur_sparse_depth = cv::Mat(down_size.height, down_size.width, CV_32F, 0.0f);
                    for (const auto& lm : landmarks[frameIdx]) {
                        Eigen::Vector3f p_W = lm.point;
                        Eigen::Vector3f p_C = T_CiW.block<3,3>(0,0) * p_W + T_CiW.block<3,1>(0,3);
                        if (p_C(2) < 0) {
                            continue;
                        }
                        Eigen::Vector3f p_norm;
                        p_norm << p_C(0)/p_C(2), p_C(1)/p_C(2), 1.0;
                        float u_proj = f_half * p_norm(0) + cu_half;
                        float v_proj = f_half * p_norm(1) + cv_half;

                        if (u_proj>0 && u_proj<down_size.width-1.0f && v_proj>0 && v_proj<down_size.height-1.0f) {
                            if (cur_sparse_depth.at<float>(static_cast<int>(v_proj), static_cast<int>(u_proj)) <= 0.0f) {
                                cur_sparse_depth.at<float>(static_cast<int>(v_proj), static_cast<int>(u_proj)) = static_cast<float>(p_C(2));
                                cv::circle(sparseDepthVis, cv::Point(2*static_cast<int>(u_proj),2*static_cast<int>(v_proj)), 6,
                                           cv::Scalar(0,static_cast<u_char>(p_C(2)/10.0*255.0),0), -1);
                                numKeypoints ++;
                            }
                        }
                    }
                }
            }

            // Make input
            // cur_image:(H,W), cur_world_T_cam: (4,4), cur_invK:(4,4)
            auto cur_image = torch::empty({height, width}, options_char);
            auto cur_image_a = cur_image.accessor<uint8_t, 2>();
            auto cur_world_T_cam = torch::empty({4, 4}, options);
            auto cur_world_T_cam_a = cur_world_T_cam.accessor<float, 2>();
            auto cur_sparse = torch::empty({down_size.height, down_size.width}, options);
            auto cur_sparse_a = cur_sparse.accessor<float, 2>();
            for (size_t h = 0; h < height; h++) {
                for (size_t w = 0; w < width; w++) {
                    const int offset = (width * h + w);
                    cur_image_a[h][w] = cur_img.data[offset];
                }
            }

            for (size_t i = 0; i < 4; i++) {
                for (size_t j = 0; j < 4; j++) {
                    cur_world_T_cam_a[i][j] = cur_c2w[4 * i + j];
                }
            }

            float *fptr = cur_sparse_depth.ptr<float>(0);
            for (size_t h = 0; h < down_size.height; h++) {
                for (size_t w = 0; w < down_size.width; w++) {
                    cur_sparse_a[h][w] = ((float) fptr[down_size.width * h + w]);
                }
            }

            // src_image: (M,H,W), src_world_T_cam: (M,4,4), src_K (M,4,4) 
            auto src_image = torch::empty({src_num, height, width}, options_char);
            auto src_image_a = src_image.accessor<uint8_t, 3>();
            auto src_world_T_cam = torch::empty({src_num, 4, 4}, options);
            auto src_world_T_cam_a = src_world_T_cam.accessor<float, 3>();

            for (size_t view = 0; view < src_num; view++) {
                float const *sc2w = src_c2ws[view];
                for (size_t i = 0; i < 4; i++) {
                    for (size_t j = 0; j < 4; j++) {
                        src_world_T_cam_a[view][i][j] = sc2w[4 * i + j];
                    }
                }

                unsigned char const *simg = src_img[view].data;
                for (size_t h = 0; h < height; h++) {
                    for (size_t w = 0; w < width; w++) {
                        const int offset = (width * h + w);
                        src_image_a[view][h][w] = simg[offset];
                    }
                }
            }

            std::vector<torch::jit::IValue> inputs;
            inputs.emplace_back(cur_image.to(torch::kCUDA));
            inputs.emplace_back(src_image.to(torch::kCUDA));
            
            // intrinsic_matrix
            inputs.emplace_back(cur_K.to(torch::kCUDA));
            inputs.emplace_back(cur_invK.to(torch::kCUDA));
            inputs.emplace_back(src_K.to(torch::kCUDA));

            // cam_to_world
            inputs.emplace_back(cur_world_T_cam.to(torch::kCUDA));
            inputs.emplace_back(src_world_T_cam.to(torch::kCUDA));
            
            // sparse depth
            inputs.emplace_back(cur_sparse.to(torch::kCUDA));

            // Inference
            torch::Tensor model_output = mvsModel.forward(inputs).toTensor(); // [2,H,W]
            isMVS = true;

            mvsDepthT = model_output.index({0,torch::indexing::Slice(),torch::indexing::Slice()}); // [H,W]
            mvsSigmaT = 4.0*model_output.index({1,torch::indexing::Slice(),torch::indexing::Slice()}); // [H,W]

            windowLine.pop_front();
        }
        auto stopMVS = std::chrono::high_resolution_clock::now();

        // Depth fusion
        auto startFusion = std::chrono::high_resolution_clock::now();
        torch::Tensor fuseDepthT, fuseSigmaT;
        cv::Mat fuseDepth, fuseSigma;
        if (isMVS) {
            torch::Tensor ivar_a = 1.0/stereoSigmaT.square();
            torch::Tensor ivar_b = 1.0/mvsSigmaT.square();
            torch::Tensor var_fuse = 1.0/(ivar_a + ivar_b);
            fuseDepthT = var_fuse * (ivar_a*stereoDepthT + ivar_b*mvsDepthT);
            fuseSigmaT = var_fuse.sqrt();
            fuseDepth = tensorToCvMatFloat(fuseDepthT.to(torch::kFloat).detach().cpu());
            fuseSigma = tensorToCvMatFloat(fuseSigmaT.to(torch::kFloat).detach().cpu());

            // //
            // std::string saveLeftPath = "/usr/wiss/juja/storage/group/srl/slamAndMapping/NTNU-BWT/mission_1/cam0/reintegrate/" + timestamp_k + ".png";
            // std::string saveRightPath = "/usr/wiss/juja/storage/group/srl/slamAndMapping/NTNU-BWT/mission_1/cam1/reintegrate/" + timestamp_k + ".png";
            // std::string saveDepthPath = "/usr/wiss/juja/storage/group/srl/slamAndMapping/NTNU-BWT/mission_1/depth0/reintegrate/" + timestamp_k + ".png";
            // cv::imwrite(saveLeftPath, left_i);
            // cv::imwrite(saveRightPath, right_i);
            
            // cv::Mat saveDepthfMat = 5000.0f * fuseDepth;
            // cv::Mat saveDepthMat;
            // saveDepthfMat.convertTo(saveDepthMat, CV_16U);
            // cv::imwrite(saveDepthPath, saveDepthMat);
        }
        auto stopFusion = std::chrono::high_resolution_clock::now();

        // Integrate depth
        auto startIntegration = std::chrono::high_resolution_clock::now();
        if (isMVS) {
            se::Measurements seMeasurements = {se::Measurement{
                depthMat2Image(fuseDepth), sensor, Eigen::Isometry3f(T_WCk)}};
            se::Image<float> seSigmaImage(width, height);
            seSigmaImage = depthMat2Image(fuseSigma);
            seMeasurements.depth_sigma = &seSigmaImage;
            integrator.integrateDepth(integration_counter, seMeasurements);
            integration_counter ++;
        }
        else {
            ;
            // se::Measurements seMeasurements = {se::Measurement{
            //     depthMat2Image(stereoDepth), sensor, Eigen::Isometry3f(T_WCk)}};
            // se::Image<float> seSigmaImage(width, height);
            // seSigmaImage = depthMat2Image(stereoSigma);
            // seMeasurements.depth_sigma = &seSigmaImage;
            // integrator.integrateDepth(integration_counter, seMeasurements);
            // integration_counter ++;
        }
        auto stopIntegration = std::chrono::high_resolution_clock::now();

        // Visualize
        cv::Mat mvsDepth_vis = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat mvsSigma_vis = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat fuseDepth_vis = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat fuseSigma_vis = cv::Mat::zeros(height, width, CV_8UC3);
        if (isMVS) {
            cv::Mat input_vis, up_row, bot_row;
            std::vector<cv::Mat> rgbImages;
            for (int ii = 0; ii < 7; ii++) {
                cv::Mat tmp_ii;
                cv::cvtColor(src_img[ii], tmp_ii, cv::COLOR_GRAY2RGB);
                rgbImages.push_back(tmp_ii);
            }

            std::vector<cv::Mat> up_vec = {rgbImages[0], rgbImages[1], rgbImages[2], rgbImages[3]};
            std::vector<cv::Mat> bot_vec = {rgbImages[4], rgbImages[5], rgbImages[6], sparseDepthVis};
            cv::hconcat(up_vec, up_row);
            cv::hconcat(bot_vec, bot_row);
            cv::vconcat(up_row, bot_row, input_vis);
            cv::imshow("Source image 1~7 and Current image", input_vis);

            cv::Mat mvs_vis, fuse_vis;
            cv::Mat mvsDepth_vis = tensorToDepthVis(mvsDepthT);
            cv::Mat mvsSigma_vis = tensorToSigmaVis(mvsSigmaT);
            cv::Mat fuseDepth_vis = tensorToDepthVis(fuseDepthT);
            cv::Mat fuseSigma_vis = tensorToSigmaVis(fuseSigmaT);
            std::vector<cv::Mat> mvs_vec = {mvsDepth_vis, mvsSigma_vis};
            std::vector<cv::Mat> fuse_vec = {fuseDepth_vis, fuseSigma_vis};
            cv::vconcat(mvs_vec, mvs_vis);
            cv::vconcat(fuse_vec, fuse_vis);
            cv::hconcat(mvs_vis, fuse_vis, mvsfuse_vis);
        }

        cv::Mat stereo_vis, network_vis;
        cv::Mat stereoDepth_vis = tensorToDepthVis(stereoDepthT);
        cv::Mat stereoSigma_vis = tensorToSigmaVis(stereoSigmaT);
        std::vector<cv::Mat> stereo_vec = {stereoDepth_vis, stereoSigma_vis};
        cv::vconcat(stereo_vec, stereo_vis);
        cv::hconcat(stereo_vis, mvsfuse_vis, network_vis);
        cv::imshow("Stereo/MVS/Fusion depth/sigma [m]", network_vis);
        cv::waitKey(10);

        // Reporting
        auto stereoDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopStereo - startStereo);
        auto mvsDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopMVS - startMVS);
        auto fusionDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopFusion - startFusion);
        auto integrationDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopIntegration - startIntegration);
        std::cout << "Image index [" << frameIdx << "] Number of sparse keypoints: " << numKeypoints << ", integration_counter:" << integration_counter
            << ", stereo/mvs/fusion/integration time [ms]: " 
            << static_cast<float>(stereoDuration.count()*1e-3) << ", " << static_cast<float>(integrationDuration.count()*1e-3) << ", "
            << static_cast<float>(mvsDuration.count()*1e-3) << ", " << static_cast<float>(fusionDuration.count()*1e-3) << std::endl;

        frameIdx ++;
    }

    map.saveMesh(outputPath + "/fusion-mesh.ply");

    return 0;
}