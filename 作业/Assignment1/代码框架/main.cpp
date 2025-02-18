#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen-3.4.0/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    rotation_angle = rotation_angle / 180.0f * MY_PI;
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model <<
        std::cos(rotation_angle), -std::sin(rotation_angle), 0, 0,
        std::sin(rotation_angle), std::cos(rotation_angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //projection <<
    //    0.3f, 0.0f, 0.0f, 0.0f,
    //    0.0f, 0.3f, 0.0f, 0.0f,
    //    0.0f, 0.0f, 0.3f, 0.0f,
    //    0.0f, 0.0f, 0.0f, 1.0f
    //    ;
    //return projection;

    eye_fov = eye_fov / 180.0f * MY_PI;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    projection <<
        1.0f / aspect_ratio / std::tan(eye_fov / 2), 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f / std::tan(eye_fov / 2), 0.0f, 0.0f,
        0.0f, 0.0f, zFar / (zFar - zNear), 1.0f,
        0.0f, 0.0f, -zFar * zNear / (zFar - zNear), 0.0f
        ;
    projection.transposeInPlace();

    std::cout << projection << std::endl;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    // 拔高作业
    angle = angle / 180.0f * MY_PI;
    Eigen::Matrix4f rotation;
    float c = std::cos(angle);
    float s = std::sin(angle);
    axis.normalize();
    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    // 参考DirectX 12 3D 游戏开发实战, p56
    rotation << 
        c + (1 - c) * x * x, (1 - c)* x* y + s * z, (1 - c)* x* z - s * y, 0,
        (1 - c)* x* y - s * z, c + (1 - c) * y * y, (1 - c)* y* z + s * x, 0,
        (1 - c)* x* z + s * y, (1 - c)* y* z - s * x, c + (1 - c) * z * z, 0,
        0, 0, 0, 1;
    rotation.transposeInPlace();
    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, -9};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        //r.set_model(get_rotation({0.0f, 1.0f, 0.0f}, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        std::cout << "model matrix" << std::endl;
        std::cout << get_model_matrix(angle) << std::endl;

        std::cout << "view matrix" << std::endl;
        std::cout << get_view_matrix(eye_pos) << std::endl;

        std::cout << "view model matrix" << std::endl;
        std::cout << get_view_matrix(eye_pos) * get_model_matrix(angle) << std::endl;

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
