#include <glm/glm.hpp>
#include <naive.h>

#include <settings.h>

std::vector<size_t> cycleChoice(size_t maxEls, int cycle){
    std::vector<size_t> res;
    for (size_t i = 0; i < maxEls; i++){
        if (i % cycle == 0){
            res.push_back(i);
        }
    }
    return res;
}

getNaiveResults::getNaiveResults(Settings & settings){
    processPath = settings.keyFramesPath + "/results_Bi17" + settings.resultsPathSurfix;
    sourcesPath = processPath + "/sources";
    EAGLE::checkPath(sourcesPath);
    targetsPath = processPath + "/targets";
    EAGLE::checkPath(targetsPath);
    texturesPath = processPath + "/textures";
    EAGLE::checkPath(texturesPath);
    weightsPath = processPath + "/weights";
    EAGLE::checkPath(weightsPath);
    // make the dir to store iteration results
    resultsPath = processPath + "/results";

    final_mesh_ scannedMesh;
    pcl::PolygonMesh mesh_;
    pcl::io::loadPLYFile(settings.keyFramesPath + "/" + settings.plyFile, mesh_);

    // create a RGB point cloud
    cloud_rgb = pcl::PointCloud<pcl::PointXYZRGB>();
    // convert to PointCloud
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_rgb);

    scannedMesh.cloud_rgb = cloud_rgb;
    scannedMesh.mesh = mesh_;
//    for (auto point: cloud_rgb.points){
//        scannedMesh.v_coords.push_back(point.x);
//        scannedMesh.v_coords.push_back(point.y);
//        scannedMesh.v_coords.push_back(point.z);

//    }
    scannedMesh.v_uv_coords = {}; //somehow filled the array

    point_num = scannedMesh.mesh.cloud.width;
    mesh_num = scannedMesh.mesh.polygons.size();

    kfIndexs = cycleChoice(100, 4); // 25 images total
    kfTotal = 100;

}

/*
----------------------------------
  Projection for each pixel
----------------------------------
*/

// project the point from the camera coord to the world coordinate system
glm::vec4 getNaiveResults::cameraToWorld(glm::vec4 X_c, size_t scanner, size_t id)
{
    glm::mat4 R = all_scanners_data_[scanner][id].view_matrix_; // from world to camera
    glm::mat4 R_inv = glm::inverse(R);
    return R_inv * X_c;
}
// project the point from the world to the (id)th camera's coordinate system
glm::vec4 getNaiveResults::worldToCamera(glm::vec4 X_w, size_t scanner, size_t id)
{
    glm::mat4 R = all_scanners_data_[scanner][id].view_matrix_; // from world to camera
    return R * X_w;
}

// project a pixel to the camera coord
// TODO: Review this. How can I use imgToCamera without id.
glm::vec4 getNaiveResults::imgToCamera(int x, int y, float z, size_t scanner, size_t id)
{
    glm::mat4 P = all_scanners_data_[scanner][id].projecton_matrix_;
    glm::mat4 P_inv = glm::inverse(P);

    return P_inv * glm::vec4(x, y, z, 1);
}

// Assuming that the projection matrix is a 3x4 matrix.
glm::vec3 getNaiveResults::cameraToImg(glm::vec4 X_c, size_t scanner, size_t id)
{
    glm::mat4 P = all_scanners_data_[scanner][id].projecton_matrix_;
    return P*X_c;
}

// project a pixel or vector to the world coord system
glm::vec4 getNaiveResults::imgToWorld(int x, int y, float z, size_t scanner, size_t id, int is_point)
{
    glm::vec4 X_c = imgToCamera(x, y, z, scanner, id);
    X_c[3] = is_point * 1.0f;
    return cameraToWorld(X_c, scanner, id);
}
// project the point to the (id)th image's plane (on origin-resolution)
//   X_w is the point's world position [x_w, y_w, z_w, 1]
//   return 3*1 matrix [x_img, y_img, z_c]
glm::vec3 getNaiveResults::worldToImg(glm::vec4 X_w, size_t scanner, size_t id)
{
    glm::vec4 X_c = worldToCamera(X_w, scanner, id);
    return cameraToImg(X_c, scanner, id);
}

/*
----------------------------------------------------------------------------------
  Use Ray-Tracing to check whether the pixel is intersecting with geometry or not
  It additionaly check whether the point is visible or not.
----------------------------------------------------------------------------------
*/

void getNaiveResults::calcValidMesh()
{
    // init ray intersection
    std::vector<unsigned int> faces(mesh_num * 3);
    for( size_t i = 0; i < mesh_num; i++ ) {
        for( size_t v_i = 0; v_i < 3; v_i++ )
            faces[i * 3 + v_i] = mesh.polygons[i].vertices[v_i];
    }
    std::vector<math::Vec3f> vertices(point_num);
    for(size_t i = 0; i < point_num; i++) {
        math::Vec3f v( cloud_rgb.points[i].x, cloud_rgb.points[i].y, cloud_rgb.points[i].z );
        vertices[i] = v;
    }
    BVHTree bvhtree(faces, vertices);

    img_valid_info.clear(); // pixel_index => valid_info
    weights.clear();
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);

    for (size_t scanner: scannerVect){
        for( size_t t : kfIndexs ) {
            img_valid_info[scanner][t] = std::vector<struct valid_info>(total);
            weights[t] = cv::Mat1f( settings.imgH, settings.imgW, 0.0 );
            for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
                struct valid_info info;
                img_valid_info[scanner][t][pixel_index] = info;
            }
            calcImgValidMesh(t, scanner, bvhtree);
        }
    }
}

// using the ray intersection method to get the pixel's depth
void getNaiveResults::calcImgValidMesh(size_t img_i, size_t scanner, BVHTree &bvhtree)
{
    // calc the camera's position (in world coord)
    glm::vec4 cam_c = (glm::vec4(0, 0, 0, 1));
    glm::vec4 cam_w = cameraToWorld(cam_c, scanner, img_i);
    math::Vec3f cam_world_p(cam_w[0], cam_w[1], cam_w[2]);

    // calc the camera's direction vector (in world coord)
    glm::vec4 cam_v_c = (glm::vec4(0, 0, 1, 0));
    glm::vec4 cam_v_w = cameraToWorld(cam_v_c, scanner, img_i);
    math::Vec3f cam_world_v(cam_v_w[0], cam_v_w[1], cam_v_w[2]);
    cam_world_v = cam_world_v.normalize();

    float depth_min = FLT_MAX;
    float depth_max = 0.0;
    float d2_min = FLT_MAX;
    float d2_max = 0.0;
    float weight_min = FLT_MAX;
    float weight_max = 0.0;
    cv::Mat1f depth_f(settings.imgH, settings.imgW, 0.0f);

    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
        for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {

            int y = static_cast<int>(pixel_index) / settings.imgW;
            int x = static_cast<int>(pixel_index) % settings.imgW;
            BVHTree::Ray ray;
            ray.origin = cam_world_p;

            glm::vec4 p_V = imgToWorld(x, y, 1.0, img_i, 0);
            math::Vec3f v(p_V[0], p_V[1], p_V[2]);
            ray.dir = v.normalize();

            ray.tmin = 0.0f;
            ray.tmax = std::numeric_limits<float>::infinity();

            BVHTree::Hit hit;
            if(bvhtree.intersect(ray, &hit)) {
                struct valid_info * info = &img_valid_info[scanner][img_i][pixel_index];
                // intersection face's id: hit.idx
                // its points ids:  hit.idx * 3 + 0, hit.idx * 3 + 1, hit.idx * 3 + 2
                info->mesh_id = hit.idx;

                float depth = cam_world_v.dot(hit.t * ray.dir);
                info->depth = depth;
                depth_f.at<float>(y,x) = depth;
                if ( depth < depth_min )
                    depth_min = depth;
                if ( depth > depth_max )
                    depth_max = depth;

                math::Vec3f const & w = hit.bcoords; // cv::Vec3f( w(0), w(1), w(2) );
                size_t v1_id = mesh.polygons[info->mesh_id].vertices[0];
                size_t v2_id = mesh.polygons[info->mesh_id].vertices[1];
                size_t v3_id = mesh.polygons[info->mesh_id].vertices[2];

                float d2 = depth * depth; // (cam_world_p(0)-_x)*(cam_world_p(0)-_x) + (cam_world_p(1)-_y)*(cam_world_p(1)-_y) + (cam_world_p(2)-_z)*(cam_world_p(2)-_z);
                if ( d2 < d2_min )
                    d2_min = d2;
                if ( d2 > d2_max )
                    d2_max = d2;

                // calc normal
                cv::Vec3f n1 = vertex_normal[ v1_id ];
                cv::Vec3f n2 = vertex_normal[ v2_id ];
                cv::Vec3f n3 = vertex_normal[ v3_id ];
                math::Vec3f normal;
                normal(0) = n1(0) * w(0) + n2(0) * w(1) + n3(0) * w(2);
                normal(1) = n1(1) * w(0) + n2(1) * w(1) + n3(1) * w(2);
                normal(2) = n1(2) * w(0) + n2(2) * w(1) + n3(2) * w(2);
                normal = normal.normalize();
                math::Vec3f vert2view = -ray.dir;
                float cos_alpha = -vert2view.dot(normal); // the cos of angle between camera dir and vertex normal
                info->cos_alpha = cos_alpha;

                // calc weight
                float weight = cos_alpha * cos_alpha / d2;
                weights[img_i].at<float>(y, x) = weight;
                if ( weight > weight_max )
                    weight_max = weight;
                if ( weight < weight_min && weight > 0 )
                    weight_min = weight;
            }
    }

#pragma omp parallel for
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;
        weights[img_i].at<float>(y, x) *= d2_max;//d2_min; // normalize the distance
    }
}

/*----------------------------------------------
 *  Valid Check
 * ---------------------------------------------*/

// chech if the point is valid under current resolution
bool getNaiveResults::pointValid(int x, int y)
{
    if(x < 0 || x >= settings.imgW)
        return false;
    if(y < 0 || y >= settings.imgH)
        return false;
    return true;
}
bool getNaiveResults::pointValid(cv::Point2i p_img)
{
    return pointValid(p_img.x, p_img.y);
}
bool getNaiveResults::pointValid(cv::Point2f p_img)
{
    return pointValid(static_cast<int>(std::round(p_img.x)), static_cast<int>(std::round(p_img.y)));
}

// chech if the point can project to the position(x,y) on the (img_id)th image
//  point_z is the point's z on the (img_id)th camera
bool getNaiveResults::pointProjectionValid(float point_z, size_t scanner, size_t img_id, int x, int y)
{
    // check if the point is valid (not bigger than imgW and imgH)
    if ( !pointValid(x, y) )
        return false;
   // get the position's valid info
    size_t p_index = static_cast<size_t>(x + y * settings.imgW);
    struct valid_info * info = &img_valid_info[scanner][img_id][p_index];
    // on the background
    if ( info->depth < 0.01f )
        return false;
    // check the depth (whether the point is occluded or too close)
    if ( point_z > info->depth + 0.02f || point_z < info->depth - 0.02f )
        return false;
    if ( pointOnBoundary(scanner, img_id, x, y) )
        return false;
    // check the angle, if the angle is too small, then it's a bad projection
    if ( info->cos_alpha > -0.2f )
        return false;
    // the point can be projected to the position on img
    return true;
}

// check if the img_id's (x, y) is on the boundary of the object
bool getNaiveResults::pointOnBoundary(size_t scanner, size_t img_id, int x, int y)
{
    // for a small patch which (x,y) is its center,
    //   if some weight in it is 0, then assume the point is on boundary
    int hw = 2;
    for ( int dy = -hw; dy <= hw; dy++ ) {
        for ( int dx = -hw; dx <= hw; dx++ ) {
            int x_ = x + dx;
            int y_ = y + dy;
            if( !pointValid( x_, y_ ) )
                continue;
            size_t p_index = static_cast<size_t>(x_ + y_ * settings.imgW);
            struct valid_info * info = &img_valid_info[scanner][img_id][p_index];
            if ( info->depth < 0.01f )
                return true;

        }
    }
    return false;
}

/*
-----------------------------------------
   Main Operations
-----------------------------------------
*/

cv::Mat3b getMean(const std::vector<cv::Mat3b>& images)
{
    if (images.empty()) return cv::Mat3b();

    // Create a 0 initialized image to use as accumulator
    cv::Mat m(images[0].rows, images[0].cols, CV_64FC3);
    m.setTo(cv::Scalar(0,0,0,0));

    // Use a temp image to hold the conversion of each input image to CV_64FC3
    // This will be allocated just the first time, since all your images have
    // the same size.
    cv::Mat temp;
    for (size_t i = 0; i < images.size(); ++i)
    {
        // Convert the input images to CV_64FC3 ...
        images[i].convertTo(temp, CV_64FC3);

        // ... so you can accumulate
        m += temp;
    }

    // Convert back to CV_8UC3 type, applying the division to get the actual mean
    m.convertTo(m, CV_8U, 1. / images.size());
    return m;
}

bool getNaiveResults::checkMeshMapImg(size_t mesh_i, size_t scanner, size_t img_i, std::vector<cv::Point2i> &v_uv, float &score)
/*
 * Takes the triangle, converts it's each vertex to Img coords and stores these coords in
 v_uv variable, where each vertex index corresponds to it's UV index.
*/
{
    v_uv.clear();
    bool flag = true;
    for(size_t p_i = 0; p_i < 3; p_i++) {
        // stores vertices corresponding to x coord
        size_t v_index = mesh.polygons[mesh_i].vertices[p_i];
        // X_w denotes the X in world coordinates
        glm::vec4 X_w = glm::vec4(cloud_rgb.points[v_index].x, cloud_rgb.points[v_index].y, cloud_rgb.points[v_index].z, 1);
        glm::vec3 X_img = worldToImg(X_w, scanner, img_i); // results in (x,y,z_c) point
        int _x = static_cast<int>( round(static_cast<double>(X_img[0])) );
        int _y = static_cast<int>( round(static_cast<double>(X_img[1])) );
        v_uv[p_i] = cv::Point2i(_x, _y);
        if( !pointProjectionValid(X_img[2], scanner, img_i, _x, _y) ) {
            flag = false;
            break;
        }
        // save the score to select a view with maximum score
        size_t p_index = static_cast<size_t>(_x + _y * settings.imgW);
        struct valid_info * info = &img_valid_info[scanner][img_i][p_index];
        score += info->cos_alpha;
    }
    return flag;
}


/*
-------------------------------------------------
  Perform operations
-------------------------------------------------
*/

void getNaiveResults::performOperations(){
// img_valid_info consists of valid_info for all pixels in the image for all images
    getNaiveResults::calcValidMesh();
#pragma omp parallel for
    for (size_t scanner = 0; scanner < all_scanners_data_.size(); scanner++){

        // store each vertex's uv index at every image to avoid duplication
        //  img_index => { vertex_index => uv_index }
        std::map<size_t, std::vector<size_t>> vertex_uv_index;
        for( size_t img_i : kfIndexs )
            vertex_uv_index[img_i] = std::vector<size_t>(point_num, 0); // 0 is the invalid index of uv
        // store each mesh's info under the mtl
        //  img_index => { mesh_index => [ [point_index, uv_index], [point_index, uv_index], [point_index, uv_index] ] }
        std::map<size_t, std::vector<struct face_info>> mesh_info;
        for( size_t img_i : kfIndexs )
            mesh_info[img_i] = std::vector<struct face_info>();

        // vertex index -> (x,y) coords in UV
        std::vector<cv::Point2i> v_uv(3), v_uv_tmp(3);
        // mesh_i means here the triangle
        for( size_t mesh_i = 0; mesh_i < mesh_num; mesh_i++ ) {
            size_t img_index = kfTotal;
            float img_score_min = 10.0f, img_score;
            // Finds the image that maps the best way (min score) onto the triangle (mesh)
            for( size_t img_i : kfIndexs ) {
                img_score = 0.0f;
                if (checkMeshMapImg(mesh_i, scanner, img_i, v_uv_tmp, img_score) == true) {
                    if (img_score < img_score_min) {
                        v_uv[0] = v_uv_tmp[0];
                        v_uv[1] = v_uv_tmp[1];
                        v_uv[2] = v_uv_tmp[2];
                        img_score_min = img_score;
                        img_index = img_i;
                    }
                }
            }
            if ( img_index < kfTotal ) {
                // valid mesh, then find its 3 points color values.
                struct face_info info;
                for(size_t p_i = 0; p_i < 3; p_i++) {
                    size_t v_index = mesh.polygons[mesh_i].vertices[p_i];
                    pcl::PointXYZRGB point = cloud_rgb.points[v_index];

                    // Filling the vertex coordinates to give that to GL further
                    scannedMesh.v_coords.push_back(point.x);
                    scannedMesh.v_coords.push_back(point.y);
                    scannedMesh.v_coords.push_back(point.z);
                    scannedMesh.v_img_ptr.push_back(img_index);

                    glm::vec4 v_coords(point.x, point.y, point.z, 1);
                    glm::vec3 v_img_coords = worldToImg(v_coords, scanner, img_index);

                    cv::Mat img = all_scanners_data_[scanner][img_index].color_buffer_;
                    auto x = v_img_coords[0];
                    auto y = v_img_coords[1];

                    scannedMesh.v_img_coords.push_back(x);
                    scannedMesh.v_img_coords.push_back(y);

                    // TODO: convert the RGB8U color to DDS if needed.

                    cv::Vec3b bgrPixel = img.at<cv::Vec3b>(x, y);
                    cv::Vec3b rgbPixel;
                    cv::cvtColor(bgrPixel, rgbPixel, cv::COLOR_BGR2RGB);

                    cv::Point2f uv_coords = scannedMesh.v_uv_coords[v_index];
                    scannedMesh.scanner_texs[scanner].at<cv::Vec3b>(uv_coords.x, uv_coords.y) = rgbPixel;

                }
            }
        }
    }
    // Average 4 resulted images.
    std::vector<cv::Mat3b> images;
    for (int i = 0; i < 4; i++){
        images.push_back(scannedMesh.scanner_texs[i]);
    }
    cv::Mat3b resultingTexture = getMean(images);
    cv::imwrite("result.png", resultingTexture, {cv::IMWRITE_PNG_COMPRESSION, 9});

    // OpenGL part

    // Enable depth test
//        glEnable(GL_DEPTH_TEST);
//        // Accept fragment if it closer to the camera than the former one
//        glDepthFunc(GL_LESS);

//        GLuint VertexArrayID;
//        glGenVertexArrays(1, &VertexArrayID);
//        glBindVertexArray(VertexArrayID);

//        // Create and compile our GLSL program from the shaders
//        GLuint programID = LoadShaders( "../playground/TransformVertexShader.vertexshader", "../playground/TextureFragmentShader.fragmentshader" );

//        // Get a handle for our "MVP" uniform
//        GLuint MatrixID = glGetUniformLocation(programID, "MVP");

//        // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
//        glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
//        // Camera matrix
//        glm::mat4 View       = glm::lookAt(
//                glm::vec3(4,3,3), // Camera is at (4,3,3), in World Space
//                glm::vec3(0,0,0), // and looks at the origin
//                glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
//        );
//        // Model matrix : an identity matrix (model will be at the origin)
//        glm::mat4 Model      = glm::mat4(1.0f);
//        // Our ModelViewProjection : multiplication of our 3 matrices
//        glm::mat4 MVP        =  Projection * View * Model; // Remember, matrix multiplication is the other way around

//        // Load the texture using any two methods
//        //GLuint Texture = loadBMP_custom("../tutorial05_textured_cube/uvtemplate.bmp");
//        GLuint Texture = loadDDS("../tutorial05_textured_cube/uvtemplate.DDS");

//        // Get a handle for our "myTextureSampler" uniform
//        GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

//        // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
//        // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
//        static const GLfloat g_vertex_buffer_data[] = {
//                -1.0f,-1.0f,-1.0f,
//                -1.0f,-1.0f, 1.0f,
//                -1.0f, 1.0f, 1.0f,
//                1.0f, 1.0f,-1.0f,
//                -1.0f,-1.0f,-1.0f,
//                -1.0f, 1.0f,-1.0f,
//                1.0f,-1.0f, 1.0f,
//                -1.0f,-1.0f,-1.0f,
//                1.0f,-1.0f,-1.0f,
//                1.0f, 1.0f,-1.0f,
//                1.0f,-1.0f,-1.0f,
//                -1.0f,-1.0f,-1.0f,
//                -1.0f,-1.0f,-1.0f,
//                -1.0f, 1.0f, 1.0f,
//                -1.0f, 1.0f,-1.0f,
//                1.0f,-1.0f, 1.0f,
//                -1.0f,-1.0f, 1.0f,
//                -1.0f,-1.0f,-1.0f,
//                -1.0f, 1.0f, 1.0f,
//                -1.0f,-1.0f, 1.0f,
//                1.0f,-1.0f, 1.0f,
//                1.0f, 1.0f, 1.0f,
//                1.0f,-1.0f,-1.0f,
//                1.0f, 1.0f,-1.0f,
//                1.0f,-1.0f,-1.0f,
//                1.0f, 1.0f, 1.0f,
//                1.0f,-1.0f, 1.0f,
//                1.0f, 1.0f, 1.0f,
//                1.0f, 1.0f,-1.0f,
//                -1.0f, 1.0f,-1.0f,
//                1.0f, 1.0f, 1.0f,
//                -1.0f, 1.0f,-1.0f,
//                -1.0f, 1.0f, 1.0f,
//                1.0f, 1.0f, 1.0f,
//                -1.0f, 1.0f, 1.0f,
//                1.0f,-1.0f, 1.0f
//        };

//        // Two UV coordinates for each vertex. They were created with Blender.
//        static const GLfloat g_uv_buffer_data[] = {
//                0.000059f, 1.0f-0.000004f,
//                0.000103f, 1.0f-0.336048f,
//                0.335973f, 1.0f-0.335903f,
//                1.000023f, 1.0f-0.000013f,
//                0.667979f, 1.0f-0.335851f,
//                0.999958f, 1.0f-0.336064f,
//                0.667979f, 1.0f-0.335851f,
//                0.336024f, 1.0f-0.671877f,
//                0.667969f, 1.0f-0.671889f,
//                1.000023f, 1.0f-0.000013f,
//                0.668104f, 1.0f-0.000013f,
//                0.667979f, 1.0f-0.335851f,
//                0.000059f, 1.0f-0.000004f,
//                0.335973f, 1.0f-0.335903f,
//                0.336098f, 1.0f-0.000071f,
//                0.667979f, 1.0f-0.335851f,
//                0.335973f, 1.0f-0.335903f,
//                0.336024f, 1.0f-0.671877f,
//                1.000004f, 1.0f-0.671847f,
//                0.999958f, 1.0f-0.336064f,
//                0.667979f, 1.0f-0.335851f,
//                0.668104f, 1.0f-0.000013f,
//                0.335973f, 1.0f-0.335903f,
//                0.667979f, 1.0f-0.335851f,
//                0.335973f, 1.0f-0.335903f,
//                0.668104f, 1.0f-0.000013f,
//                0.336098f, 1.0f-0.000071f,
//                0.000103f, 1.0f-0.336048f,
//                0.000004f, 1.0f-0.671870f,
//                0.336024f, 1.0f-0.671877f,
//                0.000103f, 1.0f-0.336048f,
//                0.336024f, 1.0f-0.671877f,
//                0.335973f, 1.0f-0.335903f,
//                0.667969f, 1.0f-0.671889f,
//                1.000004f, 1.0f-0.671847f,
//                0.667979f, 1.0f-0.335851f
//        };

//        // (1) Created vertices, for the performance reasons, let's load them into GL
//        GLuint vertexbuffer;
//        glGenBuffers(1, &vertexbuffer);
//        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

//        // The same as (1)
//        GLuint uvbuffer;
//        glGenBuffers(1, &uvbuffer);
//        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//        glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

//        // Load imgs in the buffer, there are 25 * 4 imgs
//        for (size_t scanner = 0; scanner < all_scanners_data_.size(); scanner++){
//            for (size_t img_i = 0; img_i < all_scanners_data_[0].size(); img_i++){

//            }
//        }

//        do{

//            // Clear the screen
//            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//            // Use our shader
//            glUseProgram(programID);

//            // Send our transformation to the currently bound shader,
//            // in the "MVP" uniform
//            glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

//            // Bind our texture in Texture Unit 0
//            glActiveTexture(GL_TEXTURE0);
//            glBindTexture(GL_TEXTURE_2D, Texture);
//            // Set our "myTextureSampler" sampler to use Texture Unit 0
//            glUniform1i(TextureID, 0);

//            // 1rst attribute buffer : vertices
//            glEnableVertexAttribArray(0);
//            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//            glVertexAttribPointer(
//                    0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
//                    3,                  // size
//                    GL_FLOAT,           // type
//                    GL_FALSE,           // normalized?
//                    0,                  // stride
//                    (void*)0            // array buffer offset
//            );

//            // 2nd attribute buffer : UVs
//            glEnableVertexAttribArray(1);
//            glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//            glVertexAttribPointer(
//                    1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
//                    2,                                // size : U+V => 2
//                    GL_FLOAT,                         // type
//                    GL_FALSE,                         // normalized?
//                    0,                                // stride
//                    (void*)0                          // array buffer offset
//            );

//            // Draw the triangle !
//            glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles

//            glDisableVertexAttribArray(0);
//            glDisableVertexAttribArray(1);


//        } // Check if the ESC key was pressed or the window was closed
//        while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
//               glfwWindowShouldClose(window) == 0 );

//        // Cleanup VBO and shader
//        glDeleteBuffers(1, &vertexbuffer);
//        glDeleteBuffers(1, &uvbuffer);
//        glDeleteProgram(programID);
//        glDeleteTextures(1, &Texture);
//        glDeleteVertexArrays(1, &VertexArrayID);

//        // Close OpenGL window and terminate GLFW
//        glfwTerminate();
}
