#include <iostream>
#include <fstream>
#include <random>
#include <cmath>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <nanoflann.hpp>

#include <Eigen/Eigenvalues> 

constexpr float SEARCH_RADIUS = 0.6;
// TODO: set manhole size as configurable parameter. For now, Manhole size in Gazebo: (0.8m x 0.64m)
constexpr float MAJOR_LENGTH = 0.4;
constexpr float MINOR_LENGTH = 0.32;
constexpr float MAJOR_LENGTH_SQ = 0.4*0.4;
constexpr float MINOR_LENGTH_SQ = 0.32*0.32;
constexpr float THR_MANHOLE = 0.03;  // TODO: think about more reasonable way to give threshold.
constexpr size_t NUM_SUPPORT = 9;

struct geometricSemantics{
    char type; // type (-1: not determined; 0: manhole; 1: longitudinals; ...)
    uint32_t id; // id
    Eigen::Vector3f position; // center position
    Eigen::Vector3f normal_vector; // normal vector
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> edges; // 3d edges
};

Eigen::Matrix3f skewMatrix(Eigen::Vector3f a) {
    Eigen::Matrix3f R;
    R << 0.0, -a(2), a(1),
         a(2), 0.0, -a(0),
        -a(1), a(0), 0.0;
    return R;
}

bool solveRotationMatrix(const Eigen::Vector3f& a, const Eigen::Vector3f& b, Eigen::Matrix3f& R) {
    // Make sure that a and b are normal vectors.
    // This function returns R such that b = R * a.
    Eigen::Vector3f a_norm = a / a.norm();
    Eigen::Vector3f b_norm = b / b.norm();
    Eigen::Vector3f v = a_norm.cross(b_norm);
    float phi = std::acos(a_norm.dot(b_norm));

    Eigen::Matrix3f R_transpose = std::cos(phi)*Eigen::Matrix3f::Identity()
        + (1-std::cos(phi))*(v*v.transpose())
        - std::sin(phi)*skewMatrix(v);
    R = R_transpose.transpose();
    if ((b - R * a).norm() > 1e-2) {
        // std::cout << "The difference is " << (b - R * a).norm() << std::endl;
        return false;
    }

    return true;
}

bool estimatePlane(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points,
    Eigen::Vector3f& normal, Eigen::Vector3f& position) {
    if (points.size() < 3) {
        return false;
    }

    // Solve the normal equation: Ax = b.
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f ATb = Eigen::Vector3f::Zero();
    position = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < points.size(); i ++) {
        Eigen::Vector3f pt_i = points[i];
        H += pt_i*pt_i.transpose();
        ATb -= pt_i;
        position += pt_i;
    }
    normal = H.llt().solve(ATb);
    position /= points.size();

    return true;
}

bool estimateEllipse(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> points,
    const float u_0, const float v_0, const float theta_0, float& u_est, float& v_est, float& error_ellipse) {

    bool is_success = false;
    bool is_converge = false;
    const size_t num_pt = points.size();
    if (num_pt < NUM_SUPPORT) {
        return false;
    }

    Eigen::Vector3f x(u_0, v_0, theta_0);
    for (size_t iter = 0; iter < 20; iter ++) {
        float u = x(0);
        float v = x(1);
        float theta = x(2);
        Eigen::Matrix3f JTJ = Eigen::Matrix3f::Zero();
        Eigen::Vector3f JTb = Eigen::Vector3f::Zero();
        float residual = 0;
        
        for (size_t j = 0; j < num_pt; j ++) {
            float x_j = points[j](0);
            float y_j = points[j](1);
            float const0_j = (x_j-u)*std::cos(theta) - (y_j-v)*std::sin(theta);
            float const1_j = (x_j-u)*std::sin(theta) + (y_j-v)*std::cos(theta);
            float dconst0_j = -(x_j-u)*std::sin(theta) - (y_j-v)*std::cos(theta);
            float dconst1_j = (x_j-u)*std::cos(theta) - (y_j-v)*std::sin(theta);

            float Jj_0 = -2.0*std::cos(theta)*const0_j/MAJOR_LENGTH_SQ - 2.0*std::sin(theta)*const1_j/MINOR_LENGTH_SQ;
            float Jj_1 = 2.0*std::sin(theta)*const0_j/MAJOR_LENGTH_SQ - 2.0*std::cos(theta)*const1_j/MINOR_LENGTH_SQ;
            float Jj_2 = 2.0*const0_j*dconst0_j/MAJOR_LENGTH_SQ + 2.0*const1_j*dconst1_j/MINOR_LENGTH_SQ;
            float residual_j = 1.0 - ((const0_j*const0_j)/MAJOR_LENGTH_SQ + (const1_j*const1_j)/MINOR_LENGTH_SQ);

            Eigen::Matrix<float,1,3> Jj(Jj_0, Jj_1, Jj_2);
            JTJ += Jj.transpose() * Jj;
            JTb += Jj.transpose() * residual_j;
            residual += residual_j*residual_j;
        }
        error_ellipse = residual/num_pt;
        Eigen::Vector3f delta_x = JTJ.llt().solve(JTb);
        x += delta_x;

        // std::cout << "[" << iter <<  "] residual = " << residual / num_pt
        //     << ", state = (" << x(0) << ", " << x(1) << ", " << x(2)
        //     << ", stop criteria = " << delta_x.norm()/x.norm() << std::endl;

        if (delta_x.norm()/x.norm() < 1.0e-4) {
            is_converge = true;
            break;
        }
    }
    u_est = x(0);
    v_est = x(1);

    // TODO: have to have better threhold than absolute one.
    if (is_converge && error_ellipse < THR_MANHOLE) {
        is_success = true;
    }

    return is_success;
}

void savePoints(size_t id,
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points,
                std::string save_path,
                unsigned char color[3]) {
  size_t Npoints = points.size();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_ptr->width = Npoints;
  cloud_ptr->height = 1;
  cloud_ptr->is_dense = false;
  cloud_ptr->resize (cloud_ptr->width * cloud_ptr->height);
  size_t tmpCnt = 0;
  for (auto& point: cloud_ptr->points) {
    Eigen::Vector3d point_W = points[tmpCnt].cast<double>();
    point.x = point_W(0);
    point.y = point_W(1);
    point.z = point_W(2);
    point.r = color[0];
    point.g = color[1];
    point.b = color[2];

    tmpCnt ++;
  }  
  std::string saveName = save_path + "/edges_" + std::to_string(id) + ".ply";
  pcl::io::savePLYFileASCII(saveName, *cloud_ptr);
}

// Define a structure for the 3D point cloud data
struct KdPointCloud {
    struct Point {
        float x, y, z;
    };
    
    std::vector<Point> points;

    // Number of points in the point cloud
    inline size_t kdtree_get_point_count() const { return points.size(); }

    // Accessor method for the k-d tree (point cloud data)
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return points[idx].x;
        if (dim == 1) return points[idx].y;
        return points[idx].z;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};


int main(int argc, char** argv) {

    // Check if filename is provided
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_ply_file>" << std::endl;
        return -1;
    }

    // Create a PointCloud object to hold the loaded data
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the PLY file into the PointCloud object
    std::string input_ply(argv[1]);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_ply, *in_cloud_ptr) == -1)  // Fill in the cloud data
    {
        PCL_ERROR("Couldn't read the PLY file\n");
        return -1;
    }

    // Output the size of the cloud
    std::cout << "Loaded point cloud with " << in_cloud_ptr->width * in_cloud_ptr->height
        << " points." << std::endl;

    // Define random seed.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<u_char> uchar_ran(0,255);

    // Define some constants.
    size_t pos_dir = input_ply.find_last_of("/");
    if (pos_dir == std::string::npos) {
        return -1;
    }
    const std::string save_path = input_ply.substr(0, pos_dir);

    // Define the point cloud with some 3D points
    std::map<unsigned int, Eigen::Vector3f> edge_pool;
    std::vector<uint32_t> removed_ids;
    unsigned int id_edge = 0;
    KdPointCloud kd_cloud;
    for (auto& point: in_cloud_ptr->points) {
        KdPointCloud::Point point_i;
        point_i.x = point.x;
        point_i.y = point.y;
        point_i.z = point.z;
        kd_cloud.points.push_back(point_i);
        edge_pool[id_edge] = Eigen::Vector3f(point.x, point.y, point.z);
        id_edge ++;
    }

    // Create the KDTree index for the 3D points
    // The naming is very misleading, but L2_Simple_Adaptor is based on L2-SQUARED-norm.
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, KdPointCloud>,
        KdPointCloud, 3 /*dim*/> my_kd_tree_t;

    my_kd_tree_t index(3 /*dim*/, kd_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(20 /* max leaf */));

    // Loop over all edge pool.
    std::map<uint32_t, std::vector<Eigen::Vector3f,
        Eigen::aligned_allocator<Eigen::Vector3f>>> closed_edges;
    float search_radius = SEARCH_RADIUS * SEARCH_RADIUS;
    uint32_t id_edges = 0;
    while(!edge_pool.empty()) {

        std::cout << "Number of 3d edges left to test: " << edge_pool.size() << std::endl;

        auto edge_anchor = *edge_pool.begin();
        edge_pool.erase(edge_anchor.first);
        removed_ids.push_back(edge_anchor.first);
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> candidate_edges;
        candidate_edges.push_back(edge_anchor.second); // edge-id, position
        float* query_point = edge_anchor.second.data();
        float dist_to_anchor = 1.0e+4;
        float max_dist = -1;

        int cnt_tracking = 0;
        bool is_loop = true;
        while (is_loop) {
            // Search
            std::vector<nanoflann::ResultItem<uint32_t, float>> ret_match_raw;
            std::vector<nanoflann::ResultItem<uint32_t, float>> ret_match;

            // ret_match_raw is in ascending order of the distance.
            const size_t num_matches_raw =
                index.radiusSearch(&query_point[0], search_radius, ret_match_raw);

            std::cout << "[" << cnt_tracking <<"] Query point(" << edge_anchor.first << ")= [" << query_point[0] << ", " << query_point[1] << ", " << query_point[2] << "]" << std::endl;
            // for (size_t ii = 0; ii < ret_match_raw.size(); ii++) {
            //     std::cout << ret_match_raw[ii].first << ", " << std::sqrt(ret_match_raw[ii].second) << std::endl;
            // }

            // Only count matches other than candidates.
            for (size_t i = 0; i < num_matches_raw; i ++) {
                bool is_duplicate = false;
                for (size_t j = 0; j < removed_ids.size(); j ++) {  
                    if (ret_match_raw[i].first == removed_ids[j]) {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    ret_match.push_back(ret_match_raw[i]);
                }
            }

            // When there is a valid next edge.
            if (ret_match.size() > 0) {
                uint32_t tracking_id = ret_match[0].first; // The 1st one is the nearest one.
                Eigen::Vector3f tracking_edge = edge_pool[tracking_id];
                dist_to_anchor = (edge_anchor.second - tracking_edge).norm();
                query_point = tracking_edge.data(); // update the query point.
                candidate_edges.push_back(tracking_edge);
                edge_pool.erase(tracking_id);
                removed_ids.push_back(tracking_id);
                std::cout << "    Anchor-to-tracking edges: " << "[" << edge_anchor.second(0) << ", " << edge_anchor.second(1) << ", " << edge_anchor.second(2) 
                    << "] (" << edge_anchor.first 
                    << ") - [" << tracking_edge(0) << ", " << tracking_edge(1) << ", " << tracking_edge(2) << "] (" << tracking_id
                    << "), dist_to_anchor = " << dist_to_anchor << std::endl;

                if (dist_to_anchor > max_dist) {
                    max_dist = dist_to_anchor;
                }
                if (cnt_tracking > 1e+5 || max_dist > 1.0) {
                    std::cout << "Tracking failed: " << cnt_tracking << ", " << max_dist << std::endl;
                    is_loop = false;
                }
            }
            else {
                is_loop = false;
            }
            cnt_tracking ++;
        }

        // This is a set of closed 3d edges
        std::cout << "dist_to_anchor = " << dist_to_anchor << ", " 
            << "candidate_edges.size() = " << candidate_edges.size() 
            << ", max_dist = " << max_dist << std::endl;
        if (cnt_tracking > 5 && max_dist < 1.0) {
            std::cout << "!!!! Closed edge detected !!!!" << "\n" << std::endl;
            closed_edges[id_edges] = candidate_edges;
            id_edges ++;
        }
        std::cout << "========" << std::endl;
        std::cout << "\n";
    }

    // Visualize all detected closed-edges
    for (int i = 0; i < closed_edges.size(); i ++) {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> save_edges;
        auto vec_i = closed_edges[i];
        for (int j = 0; j < vec_i.size(); j ++) {
            save_edges.push_back(vec_i[j]);
        }
        unsigned char color[3];
        color[0] = uchar_ran(gen);
        color[1] = uchar_ran(gen);
        color[2] = uchar_ran(gen);
        savePoints(i, save_edges, save_path + "/closed_edges/", color);
    }

    // Plane ransac for all clusters.
    std::vector<uint32_t> outlier_ids;
    std::vector<geometricSemantics> semantics;
    for (auto it = closed_edges.begin(); it != closed_edges.end(); ++it) {
        // log(1-p)/log(1-w^3) where p=0.99 (successful rate) and w=0.5 (outlier ratio)
        const int num_iter = 35;
        unsigned int vote_best = 0;
        Eigen::Vector3f normal_best = Eigen::Vector3f::Zero();
        Eigen::Vector3f position_best = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> inliers_best;
        for (int ri = 0; ri < num_iter; ri ++) {
            size_t num_edges =  it->second.size();
            std::uniform_int_distribution<unsigned int> int_ran(0,num_edges);
            const int num_sample = 5; // number of sampled points
            std::vector<unsigned int> idx_ri;
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points_ri;
            idx_ri.push_back(int_ran(gen));

            // Need at least 3 samples, let's take 5 here.
            while (idx_ri.size() < 5) {
                unsigned int ran_i = int_ran(gen);
                bool is_duplicate = false;
                for (int i = 0; i < idx_ri.size(); i ++) {
                    if (idx_ri[i] == ran_i) {
                        is_duplicate = true;
                    }
                }
                if (!is_duplicate) {
                    idx_ri.push_back(ran_i);
                    points_ri.push_back(it->second[ran_i]);
                }
            }
            Eigen::Vector3f normal_ri, position_ri;
            estimatePlane(points_ri, normal_ri, position_ri);

            // Test with other samples
            unsigned int vote_i = 0;
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> inliers_i;
            for (int i = 0; i < num_edges; i ++) {
                Eigen::Vector3f pt_i = it->second[i];
                float cost_i = pt_i.transpose() * normal_ri + 1.0f;
                if (cost_i*cost_i < 1.0e-4) {
                    inliers_i.push_back(pt_i);
                    vote_i ++;
                }
            }

            if (vote_i > vote_best) {
                estimatePlane(inliers_i, normal_best, position_best);
                vote_best = vote_i;
                inliers_best = inliers_i;
                normal_best /= normal_best.norm();
            }
        }

        if (vote_best < 6) {
            outlier_ids.push_back(it->first);
            std::cout << "[Reject] edge id = " << it->first 
                << ", vote_best = " << vote_best 
                << ", normal_best = [" << normal_best(0) << ", " << normal_best(1) << ", " << normal_best(2)
                << "], position_best = [" << position_best(0) << ", " << position_best(1) << ", " << position_best(2) 
                << "]" << std::endl;
        }
        else {
            std::cout << "edge id = " << it->first 
                << ", vote_best = " << vote_best 
                << ", normal_best = [" << normal_best(0) << ", " << normal_best(1) << ", " << normal_best(2)
                << "], position_best = [" << position_best(0) << ", " << position_best(1) << ", " << position_best(2) 
                << "]" << std::endl;
            // Save intermediate semantics.
            geometricSemantics semantics_i;
            semantics_i.id = it->first;
            semantics_i.position = position_best;
            semantics_i.normal_vector = normal_best;
            semantics_i.edges = inliers_best;
            semantics_i.type = -1;
            semantics.push_back(semantics_i);
        }
    }

    // Remove outliers.
    for (size_t i = 0; i < outlier_ids.size(); i ++) {
        closed_edges.erase(outlier_ids[i]);
    }

    // Visualize after plane ransac
    for (int i = 0; i < closed_edges.size(); i ++) {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> save_edges;
        auto vec_i = closed_edges[i];
        for (int j = 0; j < vec_i.size(); j ++) {
            save_edges.push_back(vec_i[j]);
        }
        unsigned char color[3];
        color[0] = uchar_ran(gen);
        color[1] = uchar_ran(gen);
        color[2] = uchar_ran(gen);
        savePoints(i, save_edges, save_path + "/plane_ransac/", color);
    }

    // tmp
    // std::ofstream outFile("projects.txt"); // Open a file for writing

    // Project to the plane (z=0) and fit to an ellipse.
    const Eigen::Vector3f v_z(0.0, 0.0, 1.0);
    const float major_squared = MAJOR_LENGTH*MAJOR_LENGTH;
    const float minor_squared = MINOR_LENGTH*MINOR_LENGTH;
    for (size_t i = 0; i < semantics.size(); i ++) {
        geometricSemantics semantics_i = semantics[i];
        const Eigen::Vector3f n_i = semantics_i.normal_vector;
        Eigen::Matrix3f R_zplane;
        if (solveRotationMatrix(n_i, v_z, R_zplane)) {
            // Project to the z-plane.
            Eigen::Vector2f center_i = Eigen::Vector2f::Zero();
            std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> projects_i;
            size_t num_i = semantics_i.edges.size();
            float mean_pz = 0;
            for (size_t ii = 0; ii < num_i; ii ++) {
                Eigen::Vector3f p_ii = R_zplane * semantics_i.edges[ii];
                Eigen::Vector2f projects_ii = p_ii.head<2>();
                center_i += projects_ii;
                projects_i.push_back(projects_ii);
                mean_pz += p_ii(2);
            }
            center_i /= num_i;
            mean_pz /= num_i;

            // Ellipse fitting with offset as variables
            unsigned char color[3];
            float u_est ,v_est, error_ellipse;
            bool is_success = estimateEllipse(projects_i, center_i(0), center_i(1), 0.0, u_est, v_est, error_ellipse);
            if (is_success) {
                // Save attributes for manholes
                semantics[i].type = 1;
                semantics[i].position = R_zplane.transpose() * Eigen::Vector3f(u_est, v_est, mean_pz);
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> center_position;
                center_position.push_back(semantics[i].position);

                color[0] = uchar_ran(gen);
                color[1] = uchar_ran(gen);
                color[2] = uchar_ran(gen);
                savePoints(semantics_i.id, semantics_i.edges, save_path + "/manholes/", color);
                savePoints(semantics_i.id, center_position, save_path + "/manhole_center_position/", color);
                std::cout << "[Manhole detected] id:"  << semantics_i.id 
                    << ", error_ellipse = " << error_ellipse
                    << ", normal: " << n_i(0) << ", " << n_i(1) << ", " << n_i(2) << std::endl;
            }
            else {
                color[0] = 55;
                color[1] = 55;
                color[2] = 55;
                savePoints(semantics_i.id, semantics_i.edges, save_path + "/edges3d/", color);
                std::cout << "[Manhole Rejected] id:"  << semantics_i.id 
                    << ", error_ellipse = " << error_ellipse
                    << ", normal: " << n_i(0) << ", " << n_i(1) << ", " << n_i(2) << std::endl;
            }
        }
    }
    // outFile.close();

    return 0;
}