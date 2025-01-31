#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <nanoflann.hpp>


void savePoints(size_t id,
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points,
                std::string save_path,
                std::string type_name) {
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

    if (type_name == "edges") {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    else if (type_name == "all") {
        point.r = 255;
        point.g = 255;
        point.b = 255;
    }
    tmpCnt ++;
  }  
  std::string saveName = save_path + "/" + type_name + "_" + std::to_string(id) + ".ply";
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
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *in_cloud_ptr) == -1)  // Fill in the cloud data
    {
        PCL_ERROR("Couldn't read the PLY file\n");
        return -1;
    }

    // Output the size of the cloud
    std::cout << "Loaded point cloud with " << in_cloud_ptr->width * in_cloud_ptr->height
        << " points." << std::endl;

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

    // // Search radius example
    // const float query_point[3] = {8.7, -5.3, 3.14};
    // Eigen::Vector3f query_v3f(query_point[0], query_point[1], query_point[2]);
    // const float resolution = 0.04;
    // float search_radius = 1.5*resolution;
    // search_radius *= search_radius;
    // std::vector<nanoflann::ResultItem<uint32_t, float>> ret_matches;
    // const size_t nMatches =
    //     index.radiusSearch(&query_point[0], search_radius, ret_matches);

    // std::cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
    //         << " matches\n";
    // for (size_t i = 0; i < nMatches; i++) {
    //     uint32_t id = ret_matches[i].first;
    //     float dist = (query_v3f - edge_pool[id]).norm();
    //     std::cout << id << ": " << std::sqrt(ret_matches[i].second) << ", " << dist << std::endl;
    // }

    // Loop over all edge pool.
    std::map<uint32_t, std::vector<std::pair<uint32_t, Eigen::Vector3f>,
        Eigen::aligned_allocator<std::pair<uint32_t, Eigen::Vector3f>>>> manhole_pts;
    const float resolution = 0.04;
    float search_radius = 15.0*resolution;
    search_radius *= search_radius;
    uint32_t id_manhole = 0;
    uint32_t cnt_tmp = 0;
    while(!edge_pool.empty()) {

        std::cout << "Number of 3d edges left to test: " << edge_pool.size() << std::endl;

        auto edge_anchor = *edge_pool.begin();
        edge_pool.erase(edge_anchor.first);
        removed_ids.push_back(edge_anchor.first);
        std::vector<std::pair<uint32_t, Eigen::Vector3f>,
            Eigen::aligned_allocator<std::pair<uint32_t, Eigen::Vector3f>>> candidate_edges;
        candidate_edges.push_back(std::pair(edge_anchor.first, edge_anchor.second)); // edge-id, position
        float* query_point = edge_anchor.second.data();
        float dist_to_anchor = 1.0e+4;
        float max_dist = -1;
        float min_dist = 1.0e+4;

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
                // for (size_t ii = 0; ii < ret_match.size(); ii++) {
                //     std::cout << "ret_match: " << ret_match[ii].first << ", " << std::sqrt(ret_match[ii].second) << std::endl;
                // }

                uint32_t tracking_id = ret_match[0].first;
                Eigen::Vector3f tracking_edge = edge_pool[tracking_id];
                dist_to_anchor = (edge_anchor.second - tracking_edge).norm();
                query_point = tracking_edge.data(); // update the query point.
                candidate_edges.push_back(std::pair(tracking_id, tracking_edge));
                edge_pool.erase(tracking_id);
                removed_ids.push_back(tracking_id);
                std::cout << "    Anchor-to-tracking edges: " << "[" << edge_anchor.second(0) << ", " << edge_anchor.second(1) << ", " << edge_anchor.second(2) << "] (" << edge_anchor.first 
                    << ") - [" << tracking_edge(0) << ", " << tracking_edge(1) << ", " << tracking_edge(2) << "] (" << tracking_id
                    << "), dist_to_anchor = " << dist_to_anchor << std::endl;
                // std::cout << tracking_edge(1) << ", " << tracking_edge(2) << std::endl;

                if (dist_to_anchor > max_dist) {
                    max_dist = dist_to_anchor;
                }
                if (cnt_tracking > 10 && dist_to_anchor < min_dist) {
                    min_dist = dist_to_anchor;
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
        std::cout << "dist_to_anchor = " << dist_to_anchor << ", " << "candidate_edges.size() = " << candidate_edges.size() << ", max_dist = " << max_dist  << ", min_dist = " << min_dist << std::endl;
        if (min_dist < 0.5 && max_dist < 1.0) {
            std::cout << "!!!! Closed edge detected !!!!" << "\n" << std::endl;
            manhole_pts[id_manhole] = candidate_edges;
            id_manhole ++;
        }
        std::cout << "========" << std::endl;
        std::cout << "\n";
        
        // if (cnt_tmp >100) break;
        cnt_tmp ++;
    }

    // Visualize detected manhole edges
    for (int i = 0; i < manhole_pts.size(); i ++) {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> save_edges;
        auto vec_i = manhole_pts[i];
        for (int j = 0; j < vec_i.size(); j ++) {
            save_edges.push_back(vec_i[j].second);
        }
        savePoints(i, save_edges, "/storage/group/srl/slamAndMapping/Autoassess/gazebo/bwt_00/meshes/manholes/", "edges");
    }


    return 0;
}