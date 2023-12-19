//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

    Eigen::Vector3f lerp(float x, Eigen::Vector3f v0, Eigen::Vector3f v1) {
        return v0 + x * (v1 - v0);
    }

    Eigen::Vector3f getTextureColor(float u_img, float v_img) {
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    // 注意：纹理坐标值分别都在[0,1]中
    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;  // 从[0,1]映射到[0,width]中
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    // 
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u*width;
        float v_img = (1-v)*height;
        int u_i = u_img, v_i = v_img;
        // 注意：
        Eigen::Vector3f u00 = getTextureColor(u_i, v_i);
        Eigen::Vector3f u01 = getTextureColor(u_i, v_i+1);
        Eigen::Vector3f u10 = getTextureColor(u_i+1, v_i);
        Eigen::Vector3f u11 = getTextureColor(u_i+1, v_i+1);
        float s = u_img - u_i, t = v_img - v_i;
        Eigen::Vector3f u0 = lerp(s, u00, u10);
        Eigen::Vector3f u1 = lerp(s, u01, u11);
        return lerp(t, u0, u1);

    }


};
#endif //RASTERIZER_TEXTURE_H
