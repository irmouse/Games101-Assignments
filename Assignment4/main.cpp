#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_set>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    if(control_points.size() == 2) {
        return control_points[0] * (1-t) + control_points[1] * t;
    } else {
        std::vector<cv::Point2f> points;
        for(int i=0;i<control_points.size()-1;i++){
            // 每两个点得到一个分割点
            auto point = control_points[i] * (1-t) + control_points[i+1] * t;
            points.push_back(point);
        }
        return recursive_bezier(points, t);
    }

}

// 实现绘制贝塞尔曲线的功能
// 
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float dt = 0.001, t = 0;
    std::set<std::pair<float, float>> point_set;
    float minx = std::numeric_limits<float>::infinity(), miny = minx, maxx = 0, maxy = 0;
    while(t <= 1) {
        auto point = recursive_bezier(control_points, t);
        minx = point.x < minx ? point.x : minx;
        miny = point.y < miny ? point.y : miny;
        maxx = point.x > maxx ? point.x : maxx;
        maxy = point.y > maxy ? point.y : maxy;
        point_set.insert(std::pair<float,float>(point.x, point.y));
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        t+=dt;
    }

    for(int i=minx; i<=maxx; i++) {
        for(int j=miny; j<=maxy; j++) {
            float color = 0;
            for(int dx : {0,1}) {
                for(int dy : {0,1}) {
                    if(point_set.find(std::pair<float,float>(i+dx, j+dy))!=point_set.end()) {
                        color+=255;
                    }
                }
            }
            color /= 4;
            window.at<cv::Vec3b>(j, i)[1] = color;
        }
    }


}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
