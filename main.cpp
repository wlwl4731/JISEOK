#include <iostream>
//#include <deque>
//#include <vector>
#include "opencv2/opencv.hpp"

int image_width_ = 640;
int image_height_ = 480;
int roi_start_height_ = 370;
int roi_height_ = 40;
int canny_edge_low_threshold_ = 60;
int canny_edge_high_threshold_ = 150;
float hough_line_slope_range_ = 20.0;
int hough_threshold_ = 45;
int hough_min_line_length_ = 40;
int hough_max_line_gap_ = 10; //100
double hough_rho = 1;
double hough_theta = CV_PI / 180.0;
static const bool kLeftLane = true;
static const bool kRightLane = false; //false?
enum kHoughIndex { x_1, y_1, x_2, y_2 };


// get average m, b of lines
std::pair<float, float> get_line_params(const std::vector<cv::Vec4i>& all_lines, const std::vector<int>& lines) {
	int x1, y1, x2, y2;
	float x_sum = 0.0f, y_sum = 0.0f, m_sum = 0.0f;
	
	int lines_size = lines.size();
	if (lines_size == 0) {
		return std::pair<float, float>(0.0f, 0.0f);
	}

	for (int i = 0; i < lines_size; ++i) {
		x1 = all_lines[lines[i]][kHoughIndex::x_1],
		y1 = all_lines[lines[i]][kHoughIndex::y_1];
		x2 = all_lines[lines[i]][kHoughIndex::x_2],
		y2 = all_lines[lines[i]][kHoughIndex::y_2];

		x_sum += x1 + x2;
		y_sum += y1 + y2;
		m_sum += (float)(y2 - y1) / (x2 - x1);
	}

	float x_avg, y_avg, m, b;
	x_avg = x_sum / (lines_size * 2);
	y_avg = y_sum / (lines_size * 2);
	m = m_sum / lines_size;
	b = y_avg - m * x_avg;

	std::pair<float, float> m_and_b(m, b);
	return m_and_b;
}


// get lpos, rpos
int get_line_pos(const std::vector<cv::Vec4i>& all_lines, const std::vector<int>& lines, const bool direction) {
	float m, b;
	std::tie(m, b) = get_line_params(all_lines, lines);

	float y, pos;
	if (m == 0.0 && b == 0.0) {
		if (direction == kLeftLane) {
			pos = 0.0f;
		}
		else {
			pos = 640.0f;
		}
	}
	else {
		y = (float)roi_height_ * 0.5;
		pos = (y - b) / m;
	}
	return std::round(pos);
}


// get left lines, right lines
std::pair<std::vector<int>, std::vector<int>> divideLines(const std::vector<cv::Vec4i>& all_lines) {
	int lines_size = all_lines.size();
	std::vector<int> left_lines;
	std::vector<int> right_lines;
	left_lines.reserve(lines_size);
	right_lines.reserve(lines_size);
	int x1, y1, x2, y2;
	float slope;
	float left_line_x_sum = 0.0f;
	float right_line_x_sum = 0.0f;
	float left_x_avg, right_x_avg;

	for (int i = 0; i < lines_size; ++i) {
		x1 = all_lines[i][kHoughIndex::x_1], y1 = all_lines[i][kHoughIndex::y_1];
		x2 = all_lines[i][kHoughIndex::x_2], y2 = all_lines[i][kHoughIndex::y_2];
		if (x2 - x1 == 0) {
			slope = 0.0f;
		}
		else {
			slope = (float)(y2 - y1) / (x2 - x1);
		}
		if (-hough_line_slope_range_ <= slope && slope <= 0) {
			if (x2 < image_width_ / 2 + 110) {
				left_line_x_sum += (float)(x1 + x2) * 0.5;
				left_lines.push_back(i);
			}
		}
		else if (0 <= slope && slope <= hough_line_slope_range_) {
			if (x1 > image_width_ / 2 -110) {
				right_line_x_sum += (float)(x1 + x2) * 0.5;
				right_lines.push_back(i);
			}
		}
	}

	int left_lines_size = left_lines.size();
	int right_lines_size = right_lines.size();

	if (left_lines_size != 0 && right_lines_size != 0) {
		left_x_avg = left_line_x_sum / left_lines_size;
		right_x_avg = right_line_x_sum / right_lines_size;
		if (left_x_avg > right_x_avg) {
			left_lines.clear();
			right_lines.clear();
			std::cout << "------Invalid Path!------\n";
		}
	}

	return std::pair<std::vector<int>, std::vector<int>>(
		std::move(left_lines), std::move(right_lines));
}


// show image and return lpos, rpos
std::pair<int, int> process_image(const cv::Mat& image) {
	cv::Mat gray_image;
	cv::Mat blur_image;
	cv::Mat canny_image;
	cv::Mat roi;

	// Image Preprocessing
	cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(gray_image, blur_image, cv::Size(5, 5), 1);
	cv::Canny(blur_image, canny_image, canny_edge_low_threshold_, canny_edge_high_threshold_);
	
	roi = canny_image(cv::Rect(0, roi_start_height_, image_width_, roi_height_));
	cv::imshow("roi", roi);
	
	// Houghlines
	std::vector<cv::Vec4i> all_lines;
	cv::HoughLinesP(roi, all_lines, hough_rho, hough_theta, hough_threshold_, hough_min_line_length_, hough_max_line_gap_);

	// divide left, right lines
	if (all_lines.size() == 0) {
		return std::pair<int, int>(0, 640);
	}
	std::vector<int> left_lines, right_lines;
	std::tie(left_lines, right_lines) = std::move(divideLines(all_lines));

	// get center of lines
	int lpos = get_line_pos(all_lines, left_lines, kLeftLane);
	int rpos = get_line_pos(all_lines, right_lines, kRightLane);

	return std::pair<int, int>(lpos, rpos);
}


// draw lines
void draw_lines(const cv::Mat& img, const std::vector<cv::Vec4i>& all_lines, const std::vector<int>& left_lines, const std::vector<int>& right_lines) {
	cv::Point2i pt1, pt2;
	cv::Scalar color;
	for (int i = 0; i < left_lines.size(); ++i) {
		pt1 = cv::Point2i(
			all_lines[left_lines[i]][kHoughIndex::x_1],
			all_lines[left_lines[i]][kHoughIndex::y_1] + roi_start_height_);
		pt2 = cv::Point2i(
			all_lines[left_lines[i]][kHoughIndex::x_2],
			all_lines[left_lines[i]][kHoughIndex::y_2] + roi_start_height_);
		int r, g, b;
		r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		color = std::move(cv::Scalar(b, g, r));
		cv::line(img, pt1, pt2, color, 2);
	}
	for (int i = 0; i < right_lines.size(); ++i) {
		pt1 = cv::Point2i(
			all_lines[right_lines[i]][kHoughIndex::x_1],
			all_lines[right_lines[i]][kHoughIndex::y_1] + roi_start_height_);
		pt2 = cv::Point2i(
			all_lines[right_lines[i]][kHoughIndex::x_2],
			all_lines[right_lines[i]][kHoughIndex::y_2] + roi_start_height_);
		int r, g, b;
		r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
		color = std::move(cv::Scalar(b, g, r));
		cv::line(img, pt1, pt2, color, 2);
	}
}


// draw rectangle
void draw_rectangles(const cv::Mat& img, int lpos, int rpos, int ma_pos) {
	static cv::Scalar kCVRed(0, 0, 255);
	static cv::Scalar kCVGreen(0, 255, 0);
	static cv::Scalar kCVBlue(255, 0, 0);

	cv::rectangle(img,
		cv::Point(lpos - 5, 15 + roi_start_height_),
		cv::Point(lpos + 5, 25 + roi_start_height_),
		kCVGreen, 2);
	cv::rectangle(img,
		cv::Point(rpos - 5, 15 + roi_start_height_),
		cv::Point(rpos + 5, 25 + roi_start_height_),
		kCVGreen, 2);
	cv::rectangle(img,
		cv::Point(ma_pos - 5, 15 + roi_start_height_),
		cv::Point(ma_pos + 5, 25 + roi_start_height_),
		kCVRed, 2);
	cv::rectangle(img,
		cv::Point(image_width_ / 2 - 5,	15 + roi_start_height_),
		cv::Point(image_width_ / 2 + 5, 25 + roi_start_height_),
		kCVBlue, 2);
}



int sam_size = 100;
std::deque<int> samples_;

std::deque<int> addSample(int new_sample) {
	samples_.push_back(new_sample);
	if (samples_.size() > sam_size) {
		samples_.pop_front();
	}
	return samples_;
}

float getMovingAverage(std::deque<int> samples_) {
	int sum = 0, sample_size = samples_.size();
	for (uint8_t i = 0; i < sample_size; ++i) {
		sum += samples_[i];
	}
	return (float)sum / sample_size;
}



int main()
{
	cv::Mat frame;
	cv::Mat dst;
	cv::VideoCapture videocapture("base_camera.avi");

	if (!videocapture.isOpened()) {
		std::cerr << "Video load failed!!" << std::endl;
	}

	while (true) {
		videocapture >> frame;
		
		if (frame.empty()) {
			std::cout << "Video End!!" << std::endl;
			break;
		}
		int lpos, rpos;
		std::tie(lpos, rpos) = process_image(frame);
		//draw_lines;
		int x = (lpos + rpos) / 2;
		std::deque<int> samp = addSample(x);
		float mpos = getMovingAverage(samp);
		std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << mpos << ", error: " << mpos-x << std::endl;

		draw_rectangles(frame, lpos, rpos, x);

		cv::imshow("VIDEO", frame);

		if (cv::waitKey(1) == 27)
		{
			std::cout << "Stop Video" << std::endl;
			break;
		}
	}
	videocapture.release();
	cv::destroyAllWindows();
}