#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// 视图变换
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate; // 平移变换矩阵(将相机移动到原点)
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;
    view = translate * view;

    return view;
}

// 模型变换：这里只是将模型按照z轴旋转rotation_angle的角度
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model =
        Eigen::Matrix4f::Identity(); // 返回单位矩阵(对角元素为1，其余为0)

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    double alpha = rotation_angle / 180.0 * MY_PI;
    model(0, 0) = std::cos(alpha);
    model(0, 1) = -std::sin(alpha);
    model(1, 1) = std::cos(alpha);
    model(1, 0) = std::sin(alpha);
    return model;
}

Eigen::Matrix4f get_projection_matrix(
    float eye_fov,
    float aspect_ratio,
    float zNear,
    float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    double height = 2.0 * zNear * std::tan(eye_fov / 180 * MY_PI / 2.0),
           width = height * aspect_ratio;
    // 注意：n和f实际上都是负值
    zNear*=-1;
    zFar*=-1;
    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();
    ortho(0, 0) = 2.0 / width;
    ortho(1, 1) = 2.0 / height;
    ortho(2, 2) = 2.0 / (zNear - zFar);
    projection *= ortho;
    ortho = Eigen::Matrix4f::Identity();
    ortho(2, 3) = -(zNear + zFar) / 2.0;
    projection *= ortho;

    Eigen::Matrix4f perspToOrtho;
    perspToOrtho << zNear, 0, 0, 0, 
    0, zNear, 0, 0, 0, 
    0, zNear + zFar, -zNear * zFar, 
    0, 0, 1, 0;

    projection *= perspToOrtho;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false; // 是否使用命令行模式
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) { filename = std::string(argv[3]); }
    }

    // 光栅化器
    rst::rasterizer r(700, 700);         // 分辨率

    Eigen::Vector3f eye_pos = {0, 0, 5}; // 相机位置

    // 三个顶点，表示三角形
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 索引数组，表示三角形由哪三个顶点组成
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0; // 帧计数器，用来显示程序性能

    if (command_line) {
        // 清空颜色缓冲和深度缓冲
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 模型矩阵
        r.set_model(get_model_matrix(angle));
        // 视图矩阵
        r.set_view(get_view_matrix(eye_pos));
        // 投影矩阵，视场角+宽高比+近平面+远平面
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        // Mat对象，用来存储和处理图像数据。将颜色缓冲区的数据复制到Mat对象中
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) { // 27为ESC键的ascii码
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10); // 10表示等待的毫秒数，即每10ms就刷新一次窗口

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        } else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
