#include "../include/udp_connection/laptop/vision_node.hpp"

VisionNode::VisionNode() :  yellow_line_detected(false), white_line_detected(false), yellow_line_count(0), white_line_count(0), array_index(0), yellow_line_valid(false), white_line_valid(false), yellow_line_x(0.0), white_line_x(0.0)
{
    node = rclcpp::Node::make_shared("vision_node");

    RCLCPP_INFO(node->get_logger(), "vision_node 초기화 완료");

    initialized_ = true;

    // ========== [서브스크라이브, 퍼블리시] ==========
    sub_image_ = node->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

    pub_original_ = node->create_publisher<sensor_msgs::msg::Image>("/vision/original", 10);
    pub_yellow_mask_ = node->create_publisher<sensor_msgs::msg::Image>("/vision/yellow_mask", 10);
    pub_white_mask_ = node->create_publisher<sensor_msgs::msg::Image>("/vision/white_mask", 10);
    pub_line_ = node->create_publisher<sensor_msgs::msg::Image>("/vision/line_detection", 10);

    pub_yellow_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10);
    pub_white_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/white_line_detected", 10);

    pub_yellow_pos_ = node->create_publisher<std_msgs::msg::Float32>("/vision/yellow_line_x", 10);
    pub_white_pos_ = node->create_publisher<std_msgs::msg::Float32>("/vision/white_line_x", 10);

    pub_blue_sign_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/blue_sign_detected", 10);

    pub_white_line_points_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/vision/white_line_points", 10);
    pub_yellow_line_points_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/vision/yellow_line_points", 10);

    pub_yellow_angle_ = node->create_publisher<std_msgs::msg::Float32>("/vision/yellow_line_angle", 10);
    pub_white_angle_ = node->create_publisher<std_msgs::msg::Float32>("/vision/white_line_angle", 10);

    pub_barrier_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/barrier_detected", 10);

    pub_barrier_yellow_line_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/barrier_yellow_line_detected", 10);
    pub_barrier_white_line_detected_ = node->create_publisher<std_msgs::msg::Bool>("/vision/barrier_white_line_detected", 10);
    pub_barrier_yellow_line_angle_ = node->create_publisher<std_msgs::msg::Float32>("/vision/barrier_yellow_line_angle", 10);
    pub_barrier_white_line_angle_ = node->create_publisher<std_msgs::msg::Float32>("/vision/barrier_white_line_angle", 10);

    pub_yellow_center_dist_ = node->create_publisher<std_msgs::msg::Float32>("/vision/yellow_line_center_dist", 10);
    pub_white_center_dist_ = node->create_publisher<std_msgs::msg::Float32>("/vision/white_line_center_dist", 10);

    yellow_detection_array.fill(false);
    white_detection_array.fill(false);
}

VisionNode::~VisionNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void VisionNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool VisionNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}


// ========== [Vision 라인 감지 처리] ==========
bool VisionNode::isLineValid(std::array<bool, 10> &detection_array, bool current_detection)
{
    // 현재 감지 결과를 배열에 저장
    detection_array[array_index] = current_detection;
    // true의 개수 계산
    int detection_count = 0;
    for (bool detection : detection_array) {
        if (detection) {
            detection_count++;
        }
    }
    // 임계값 이상이면 유효한 선으로 판단
    return detection_count >= DETECTION_THRESHOLD;
}


// ========== [Vision 처리] ==========
void VisionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(640, 480));

        int height = resized_frame.rows;
        int width = resized_frame.cols;

        // ROI 설정
        cv::Point2f src_vertices[4];
        cv::Point2f dst_vertices[4];
        cv::Point2f signs_vertices[4];
        cv::Point2f bar_vertices[4];
        
        // 차단바
        bar_vertices[0] = cv::Point2f(width * 0.35f, height * 0.55f);
        bar_vertices[1] = cv::Point2f(width * 0.65f, height * 0.55f);
        bar_vertices[2] = cv::Point2f(width * 0.65f, height * 0.7f);
        bar_vertices[3] = cv::Point2f(width * 0.35f, height * 0.7f);

        // 표지판
        signs_vertices[0] = cv::Point2f(width * 0.95f, height * 0.3f);
        signs_vertices[1] = cv::Point2f(width * 1.0f, height * 0.3f);
        signs_vertices[2] = cv::Point2f(width * 1.0f, height * 0.55f);
        signs_vertices[3] = cv::Point2f(width * 0.95f, height * 0.55f);

        // 라인트레이싱
        src_vertices[0] = cv::Point2f(width * 0.1f, height * 0.9f);
        src_vertices[1] = cv::Point2f(width * 0.9f, height * 0.9f);
        src_vertices[2] = cv::Point2f(width * 0.95f, height * 1.0f);
        src_vertices[3] = cv::Point2f(width * 0.05f, height * 1.0f);

        dst_vertices[0] = cv::Point2f(0, 0);
        dst_vertices[1] = cv::Point2f(width, 0);
        dst_vertices[2] = cv::Point2f(width, height);
        dst_vertices[3] = cv::Point2f(0, height);

        // 버드아이뷰 변환
        cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
        cv::Mat birds_eye_view;
        cv::warpPerspective(resized_frame, birds_eye_view, perspective_matrix, cv::Size(width, height));

        // 전처리 과정
        // cv::Mat birds_eye_view;
        // cv::GaussianBlur(birds_eye_view, preprocessed, cv::Size(5, 5), 0);

        // CLAHE 적용 (L*a*b* 색공간)
        cv::Mat lab;
        cv::cvtColor(birds_eye_view, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.5, cv::Size(8, 8));
        clahe->apply(lab_channels[0], lab_channels[0]);

        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, birds_eye_view, cv::COLOR_Lab2BGR);

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        cv::Mat hsv;
        cv::cvtColor(birds_eye_view, hsv, cv::COLOR_BGR2HSV);

        // 노란색 HSV,Lab, RGB 3가지로 색을 받아서 혼합.
        cv::Mat yellow_mask_combined;

        cv::Mat yellow_mask_hsv;
        cv::Scalar lower_yellow_hsv(15, 120, 120);
        cv::Scalar upper_yellow_hsv(30, 255, 255);
        cv::inRange(hsv, lower_yellow_hsv, upper_yellow_hsv, yellow_mask_hsv);

        cv::Mat yellow_mask_lab;
        cv::inRange(lab, cv::Scalar(150, 130, 140), cv::Scalar(250, 140, 200), yellow_mask_lab);

        cv::Mat yellow_mask_rgb;
        cv::inRange(birds_eye_view, cv::Scalar(180, 180, 0), cv::Scalar(255, 255, 150), yellow_mask_rgb);

        cv::bitwise_or(yellow_mask_hsv, yellow_mask_lab, yellow_mask_combined);
        cv::bitwise_or(yellow_mask_combined, yellow_mask_rgb, yellow_mask_combined);

        // 모폴로지 연산, 커널 사이즈 지정
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31));

        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_CLOSE, kernel_large);

        // 흰색 HSV, Lab, RGB 3가지로 색을 받아서 혼합.
        cv::Mat white_mask_combined;

        // 1. HSV 기반 흰색 검출
        cv::Mat white_mask_hsv;
        cv::Scalar lower_white_hsv(0, 0, 160);
        cv::Scalar upper_white_hsv(180, 20, 255);
        cv::inRange(hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);

        // 2. Lab 기반 흰색 검출
        cv::Mat white_mask_lab;
        std::vector<cv::Mat> lab_channels_white;
        cv::split(lab, lab_channels_white);
        cv::threshold(lab_channels_white[0], white_mask_lab, 235, 255, cv::THRESH_BINARY);

        // 3. RGB 기반 흰색 검출
        cv::Mat white_mask_rgb;
        cv::inRange(birds_eye_view, cv::Scalar(240, 240, 240), cv::Scalar(255, 255, 255), white_mask_rgb);

        // 노란색 마스크 제외, dilate로 노이즈 조금 제거
        cv::Mat yellow_mask_dilated;
        cv::dilate(yellow_mask_combined, yellow_mask_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        // or 연산
        cv::bitwise_or(white_mask_hsv, white_mask_lab, white_mask_combined);
        cv::bitwise_or(white_mask_combined, white_mask_rgb, white_mask_combined);
        cv::bitwise_and(white_mask_combined, ~yellow_mask_dilated, white_mask_combined);

        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_CLOSE, kernel_large);
        cv::dilate(white_mask_combined, white_mask_combined, kernel, cv::Point(-1, -1), 2);

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        //  선 검출 (컨투어 기반)
        std::vector<std::vector<cv::Point>> yellow_contours, white_contours;
        cv::findContours(yellow_mask_combined, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(white_mask_combined, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 필터링 및 선 그리기
        cv::Mat line_display = birds_eye_view.clone();

        // 기본 설정
        yellow_line_detected = false;
        white_line_detected = false;
        yellow_line_count = 0;
        white_line_count = 0;

        // 노란색 선 검출 부분
        std::vector<cv::Vec4i> yellow_lines;
        cv::HoughLinesP(yellow_mask_combined, yellow_lines, 1, CV_PI / 180, 30, 20, 40);

        if (!yellow_lines.empty()) {
            yellow_line_detected = true;
            yellow_line_count = yellow_lines.size();

            // 가장 외곽(왼쪽) 라인 찾기
            float leftmost_x = width;
            cv::Vec4i leftmost_line;
            bool found = false;

            for (const auto &line : yellow_lines) {
                float x1 = line[0], x2 = line[2];
                float avg_x = (x1 + x2) / 2;
                if (avg_x < leftmost_x)
                {
                    leftmost_x = avg_x;
                    leftmost_line = line;
                    found = true;
                }
            }

            if (found) {
                float x1 = leftmost_line[0], y1 = leftmost_line[1];
                float x2 = leftmost_line[2], y2 = leftmost_line[3];

                float mid_x = (x1 + x2) / 2;
                float distance_from_center = mid_x - (width / 2);

                yellow_line_x = (leftmost_x / width) * 2 - 1;

                // 각도 계산 수정
                float dy = y1 - y2;
                float dx = x1 - x2;
                float angle = std::atan2(dy, dx) * 180.0f / M_PI;

                // 수직선에 가까운 각도로 변환 (90도에 가깝게)
                if (angle < 0) {
                    angle += 180.0f;
                }

                // 시각화를 위한 선 그리기
                cv::line(line_display, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 2);

                // 좌표 전송 추가 (y축 반전)
                auto line_points_msg = std_msgs::msg::Float32MultiArray();
                line_points_msg.data.resize(2);
                line_points_msg.data[0] = x1;
                line_points_msg.data[1] = height - y1;
                pub_yellow_line_points_->publish(line_points_msg);

                auto yellow_pos_msg = std_msgs::msg::Float32();
                yellow_pos_msg.data = yellow_line_x;
                pub_yellow_pos_->publish(yellow_pos_msg);

                auto angle_msg = std_msgs::msg::Float32();
                angle_msg.data = angle;
                pub_yellow_angle_->publish(angle_msg);

                auto center_dist_msg = std_msgs::msg::Float32();
                center_dist_msg.data = distance_from_center;
                pub_yellow_center_dist_->publish(center_dist_msg);
            }
        }


        
        // 흰색 선 검출 부분
        std::vector<cv::Vec4i> white_lines;
        cv::HoughLinesP(white_mask_combined, white_lines, 1, CV_PI / 180, 30, 20, 10);

        if (!white_lines.empty())
        {
            white_line_detected = true;
            white_line_count = white_lines.size();

            // 가장 외곽(오른쪽) 라인 찾기
            float rightmost_x = 0;
            cv::Vec4i rightmost_line;
            bool found = false;

            for (const auto &line : white_lines)
            {
                float x1 = line[0], x2 = line[2];
                float avg_x = (x1 + x2) / 2;
                if (avg_x > rightmost_x)
                {
                    rightmost_x = avg_x;
                    rightmost_line = line;
                    found = true;
                }
            }

            if (found)
            {
                float x1 = rightmost_line[0], y1 = rightmost_line[1];
                float x2 = rightmost_line[2], y2 = rightmost_line[3];

                float mid_x = (x1 + x2) / 2;
                float distance_from_center = mid_x - (width / 2);

                white_line_x = (rightmost_x / width) * 2 - 1;

                // 각도 계산 수정
                float dy = y2 - y1;
                float dx = x2 - x1;
                float angle = std::atan2(dy, dx) * 180.0f / M_PI;

                // 수직선에 가까운 각도로 변환 (90도에 가깝게)
                if (angle < 0)
                {
                    angle += 180.0f;
                }

                cv::line(line_display, cv::Point(x1, y1), cv::Point(x2, y2),
                         cv::Scalar(255, 255, 255), 2);

                // 좌표 전송 추가 (y축 반전)
                auto line_points_msg = std_msgs::msg::Float32MultiArray();
                line_points_msg.data.resize(2);
                line_points_msg.data[0] = x1;
                line_points_msg.data[1] = height - y1;
                pub_white_line_points_->publish(line_points_msg);

                auto white_pos_msg = std_msgs::msg::Float32();
                white_pos_msg.data = white_line_x;
                pub_white_pos_->publish(white_pos_msg);

                auto angle_msg = std_msgs::msg::Float32();
                angle_msg.data = angle;
                pub_white_angle_->publish(angle_msg);

                auto center_dist_msg = std_msgs::msg::Float32();
                center_dist_msg.data = distance_from_center;
                pub_white_center_dist_->publish(center_dist_msg);
            }
        }

        if (yellow_line_detected || white_line_detected) // 노란선 감지 또는 흰 선감지
        {
            // nothing here
        }

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        // 차단바 검출
        cv::Mat bar_roi_mask = cv::Mat::zeros(resized_frame.size(), CV_8UC1);
        std::vector<cv::Point> bar_roi_points;
        for (int i = 0; i < 4; i++)
        {
            bar_roi_points.push_back(cv::Point(bar_vertices[i].x, bar_vertices[i].y));
        }
        cv::fillConvexPoly(bar_roi_mask, bar_roi_points, cv::Scalar(255));

        // 차단바용 색상 검출
        cv::Mat bar_hsv;
        cv::cvtColor(resized_frame, bar_hsv, cv::COLOR_BGR2HSV);

        // 차단바 검출을 위한 노란색 마스크
        cv::Mat bar_yellow_roi;
        cv::Scalar barrier_lower_yellow(15, 100, 100);
        cv::Scalar barrier_upper_yellow(35, 255, 255);
        cv::inRange(bar_hsv, barrier_lower_yellow, barrier_upper_yellow, bar_yellow_roi);

        // 차단바 영역에서 라인 검출 추가
        cv::Mat bar_yellow_line_mask, bar_white_line_mask;
        cv::inRange(bar_hsv, barrier_lower_yellow, barrier_upper_yellow, bar_yellow_line_mask);
        cv::inRange(bar_hsv, lower_white_hsv, upper_white_hsv, bar_white_line_mask);

        // ROI 적용
        cv::bitwise_and(bar_yellow_line_mask, bar_roi_mask, bar_yellow_line_mask);
        cv::bitwise_and(bar_white_line_mask, bar_roi_mask, bar_white_line_mask);

        // 노이즈 제거
        cv::morphologyEx(bar_yellow_line_mask, bar_yellow_line_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_yellow_line_mask, bar_yellow_line_mask, cv::MORPH_CLOSE, kernel_large);
        cv::morphologyEx(bar_white_line_mask, bar_white_line_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_white_line_mask, bar_white_line_mask, cv::MORPH_CLOSE, kernel_large);

        // ROI 적용
        cv::bitwise_and(bar_yellow_roi, bar_roi_mask, bar_yellow_roi);

        // 노이즈 제거
        cv::morphologyEx(bar_yellow_roi, bar_yellow_roi, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(bar_yellow_roi, bar_yellow_roi, cv::MORPH_CLOSE, kernel_large);

        bool barrier_yellow_detected = false;
        bool barrier_white_detected = false;
        float barrier_yellow_angle = 0.0f;
        float barrier_white_angle = 0.0f;

        // 라인 컨투어 검출
        std::vector<std::vector<cv::Point>> bar_yellow_line_contours, bar_white_line_contours;
        cv::findContours(bar_yellow_line_mask, bar_yellow_line_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(bar_white_line_mask, bar_white_line_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 차단바 검출을 위한 컨투어 처리
        std::vector<std::vector<cv::Point>> bar_yellow_contours;
        cv::findContours(bar_yellow_roi, bar_yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> candidate_rects;
        barrier_detected = false;

        // 먼저 적절한 크기의 모든 사각형을 수집
        for (const auto &yellow_contour : bar_yellow_contours)
        {
            double area = cv::contourArea(yellow_contour);
            if (area > 50.0 && area < 1000.0)
            {
                cv::RotatedRect yellow_rect = cv::minAreaRect(yellow_contour);
                candidate_rects.push_back(yellow_rect);
            }
        }

        if (candidate_rects.size() >= 4)
        {
            // y 좌표로 정렬, 일직선상의 사각형들이 4개 이상
            std::sort(candidate_rects.begin(), candidate_rects.end(),
                      [](const cv::RotatedRect &a, const cv::RotatedRect &b)
                      {
                          return a.center.y < b.center.y;
                      });

            float prev_y = candidate_rects[0].center.y;
            int same_line_count = 1;
            float total_width = 0;
            float avg_height = 0;

            for (size_t i = 1; i < candidate_rects.size(); i++)
            {
                float curr_y = candidate_rects[i].center.y;
                if (std::abs(curr_y - prev_y) < 15.0)
                {
                    same_line_count++;
                    float width = std::min(candidate_rects[i].size.width, candidate_rects[i].size.height);
                    float height = std::max(candidate_rects[i].size.width, candidate_rects[i].size.height);
                    total_width += width;
                    avg_height += height;

                    float x_diff = std::abs(candidate_rects[i].center.x - candidate_rects[i - 1].center.x);
                    if (x_diff > 50.0)
                        continue;
                }
            }

            avg_height /= same_line_count;

            if (same_line_count >= 4 && avg_height > 10.0 && total_width > 50.0)
            {
                barrier_detected = true;

                // 시각화
                for (const auto &rect : candidate_rects)
                {
                    cv::Point2f vertices[4];
                    rect.points(vertices);
                    for (int i = 0; i < 4; i++)
                    {
                        cv::line(line_display, vertices[i], vertices[(i + 1) % 4],
                                 cv::Scalar(0, 0, 255), 2);
                    }
                }
            }
        }

        auto barrier_yellow_detected_msg = std_msgs::msg::Bool();
        barrier_yellow_detected_msg.data = barrier_yellow_detected;
        pub_barrier_yellow_line_detected_->publish(barrier_yellow_detected_msg);

        // 흰색 라인 검출 결과 publish
        auto barrier_white_detected_msg = std_msgs::msg::Bool();
        barrier_white_detected_msg.data = barrier_white_detected;
        pub_barrier_white_line_detected_->publish(barrier_white_detected_msg);

        // 노란색 라인 각도 publish (라인이 검출된 경우에만)
        if (barrier_yellow_detected)
        {
            auto barrier_yellow_angle_msg = std_msgs::msg::Float32();
            barrier_yellow_angle_msg.data = barrier_yellow_angle;
            pub_barrier_yellow_line_angle_->publish(barrier_yellow_angle_msg);
        }

        // 흰색 라인 각도 publish (라인이 검출된 경우에만)
        if (barrier_white_detected)
        {
            auto barrier_white_angle_msg = std_msgs::msg::Float32();
            barrier_white_angle_msg.data = barrier_white_angle;
            pub_barrier_white_line_angle_->publish(barrier_white_angle_msg);
        }
        // 차단바 검출 결과 펍
        auto barrier_msg = std_msgs::msg::Bool();
        barrier_msg.data = barrier_detected;
        pub_barrier_detected_->publish(barrier_msg);
        
        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
        
        // 표지판 ROI 마스크 생성
        cv::Mat sign_roi_mask = cv::Mat::zeros(resized_frame.size(), CV_8UC1);
        std::vector<cv::Point> roi_points;
        for (int i = 0; i < 4; i++)
        {
            roi_points.push_back(cv::Point(signs_vertices[i].x, signs_vertices[i].y));
        }
        cv::fillConvexPoly(sign_roi_mask, roi_points, cv::Scalar(255));

        // 먼저 ROI 영역만 추출
        cv::Mat roi_image;
        resized_frame.copyTo(roi_image, sign_roi_mask);

        // ROI 영역에 대해서만 HSV 변환 수행
        cv::Mat sign_hsv;
        cv::cvtColor(roi_image, sign_hsv, cv::COLOR_BGR2HSV);

        // 파란색 마스크 생성 - HSV 값 조정
        cv::Mat blue_mask;
        cv::Scalar lower_blue_hsv(100, 70, 50);
        cv::Scalar upper_blue_hsv(130, 255, 255);
        cv::inRange(sign_hsv, lower_blue_hsv, upper_blue_hsv, blue_mask);

        // 노이즈 제거
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel_large);

        // 파란색 영역 검출
        std::vector<std::vector<cv::Point>> blue_contours;
        cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool blue_sign_detected = false;
        for (const auto &contour : blue_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 100.0)
            {
                blue_sign_detected = true;
                break;
            }
        }

        // 파란색 표지판 검출 결과 발행
        auto blue_sign_msg = std_msgs::msg::Bool();
        blue_sign_msg.data = blue_sign_detected;
        pub_blue_sign_detected_->publish(blue_sign_msg);

        // ROI 영역 표시
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, bar_vertices[i], bar_vertices[(i + 1) % 4],
                     cv::Scalar(255, 0, 0), 2);
        }
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, signs_vertices[i], signs_vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }

        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, src_vertices[i], src_vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        // 표지판 ROI 영역 표시 (빨간색으로 표시)
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, signs_vertices[i], signs_vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }

        yellow_line_valid = isLineValid(yellow_detection_array, yellow_line_detected);
        white_line_valid = isLineValid(white_detection_array, white_line_detected);

        array_index = (array_index + 1) % ARRAY_SIZE;

        auto yellow_detected_msg = std_msgs::msg::Bool();
        auto white_detected_msg = std_msgs::msg::Bool();

        yellow_detected_msg.data = yellow_line_valid;
        white_detected_msg.data = white_line_valid;

        pub_yellow_detected_->publish(yellow_detected_msg);
        pub_white_detected_->publish(white_detected_msg);

        sensor_msgs::msg::Image::SharedPtr original_msg =
            cv_bridge::CvImage(msg->header, "bgr8", resized_frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr yellow_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", yellow_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr white_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", white_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr line_msg =
            cv_bridge::CvImage(msg->header, "bgr8", line_display).toImageMsg();

        pub_original_->publish(*original_msg);
        pub_yellow_mask_->publish(*yellow_mask_msg);
        pub_white_mask_->publish(*white_mask_msg);
        pub_line_->publish(*line_msg);

        // QImage로 변환 후 QPixmap을 생성하여 원본 이미지를 전송
        QImage qImageResizedFrame(resized_frame.data, resized_frame.cols, resized_frame.rows, resized_frame.step, QImage::Format_RGB888);
        QImage qImageDetectedFrame(line_display.data, line_display.cols, line_display.rows, line_display.step, QImage::Format_RGB888);
        QImage qImageYellowMaskFrame(yellow_mask_combined.data, yellow_mask_combined.cols, yellow_mask_combined.rows, yellow_mask_combined.step, QImage::Format_Grayscale8);
        QImage qImageWhiteMaskFrame(white_mask_combined.data, white_mask_combined.cols, white_mask_combined.rows, white_mask_combined.step, QImage::Format_Grayscale8);
        
        QPixmap pixmapResized = QPixmap::fromImage(qImageResizedFrame.rgbSwapped());  // RGB로 변환 후 QPixmap 생성
        QPixmap pixmapDetected = QPixmap::fromImage(qImageDetectedFrame.rgbSwapped());
        QPixmap pixmapYellowMask = QPixmap::fromImage(qImageYellowMaskFrame.rgbSwapped());
        QPixmap pixmapWhiteMask = QPixmap::fromImage(qImageWhiteMaskFrame.rgbSwapped());

        emit imageReceived(pixmapResized, pixmapDetected, pixmapYellowMask, pixmapWhiteMask);  // 원본 이미지를 전송
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_INFO(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
