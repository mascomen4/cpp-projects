#ifndef NAIVE_H
#define NAIVE_H

#include <stdio.h>
#include <string>
#include <vector>
#include <omp.h>
#include <cfloat>
#include <time.h>
#include <sys/time.h>
#include <map>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include </home/ivan/ivan/git/ogl-master/external/glfw-3.1.2/include/GLFW/glfw3.h>

#include <shader.hpp>
#include <texture.hpp>

#include <settings.h>
#include <acc/bvh_tree.h>

#include <opencv2/core.hpp>

struct data{
    cv::Mat z_buffer_;
    cv::Mat color_buffer_; // RGB8U
    glm::mat4 view_matrix_;
    glm::mat4 projecton_matrix_;
};

struct final_mesh_{
    // Here we store the points XYZ
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;

    // Here we store the id of the points
    pcl::PolygonMesh mesh;

    // Normals for each vertex
    std::vector<glm::vec4> normals;

    std::vector<float> v_coords; // size = p_num * 3

    // {point_i => it's uv_coords}, for OpenGL
    std::vector<float> v_uv_coordsGL; //size = p_num * 2

    std::vector<cv::Point2f> v_uv_coords;

    // 4 textures, each one from the corresponding scanner
    std::vector<cv::Mat> scanner_texs;

    std::vector<size_t>  v_img_ptr; // size = p_num {can point from 0 to 100}

    std::vector<float> v_img_coords; // size = p_num * 2

    // Texture Atlas.
    cv::Mat tex;
};

std::vector<std::vector<data>> all_scanners_data_(4);

class getNaiveResults{
public:
    Settings settings;

    // Every function peforms per pixel transformation
    // X_c means X camera coords and X_w means X world coords.
    glm::vec4 cameraToWorld(glm::vec4 X_c, size_t scanner, size_t id);
    glm::vec4 worldToCamera(glm::vec4 X_w, size_t scanner, size_t id);
    glm::vec4 imgToCamera(int x, int y, float z, size_t scanner, size_t id);
    glm::vec3 cameraToImg(glm::vec4 X_c, size_t scanner, size_t id);
    glm::vec4 imgToWorld(int x, int y, float z, size_t scanner, size_t id, int is_point = 1);
    glm::vec3 worldToImg(glm::vec4 X_w, size_t scanner, size_t id);

    typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;
    bool checkMeshMapImg(size_t mesh_i, size_t scanner, size_t img_i, std::vector<cv::Point2i> &v_uv, float &score);
    void calcImgValidMesh(size_t img_i, size_t scanner, BVHTree &bvhtree);
    void calcValidMesh();
    bool pointValid(int x, int y);
    bool pointValid(cv::Point2i p_img);
    bool pointValid(cv::Point2f p_img);
    bool pointProjectionValid(float point_z, size_t scanner, size_t img_id, int x, int y);
    bool pointProjectionValidMesh(float point_z, size_t img_id, int x, int y);
    bool pointOnBoundary(size_t scanner, size_t img_id, int x, int y);

    struct valid_info // a pixel's valid info of the mesh
    {
        float depth = 0; // 0m ~ 1m
        float cos_alpha = 0;
        size_t mesh_id = 0;
    };

    std::vector<int> scannerVect {0,1,2,3};

    std::string processPath, resultsPath;
    std::string sourcesPath, targetsPath, texturesPath, weightsPath;

    size_t kfStart, kfTotal;
    std::vector<std::map<size_t, std::vector<struct valid_info>>> img_valid_info;
    std::map<size_t, cv::Mat> weights;
    std::map<size_t, cv::Mat> img_valid_patch;
    std::map<size_t, std::map<size_t, cv::Mat>> mappings;
    std::vector<std::size_t> kfIndexs;

    final_mesh_ scannedMesh;
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    size_t point_num, mesh_num;
    std::vector<cv::Vec3f> vertex_normal;

    std::map<size_t, cv::Vec3b> v_index_color;

    struct face_info
    {
        std::vector<size_t> v_index = std::vector<size_t>(3);
        std::vector<size_t> uv_index = std::vector<size_t>(3);
        std::vector<size_t> n_index = std::vector<size_t>(3);
    };

    // OpenGL types
    static const GLfloat g_vertex_buffer_data[];
    static const GLfloat g_vertexUV_buffer_data[];
    static const GLfloat g_verteximgPtr_buffer_data[];
    static const GLfloat g_vertexImg_coords_buffer_data[];



    getNaiveResults(Settings &_settings);
    ~getNaiveResults();

    void performOperations();

    int nImg = 100;
    int nScanners = 4;
};


#endif // NAIVE_H
