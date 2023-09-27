#pragma once // Header guard to prevent multiple inclusions

#include <opencv2/opencv.hpp> 

namespace edge_detection {

class EdgeDetector {
public:
    // Function to detect edges in an input image and store the result in the output image
    void detectEdges(const cv::Mat& inputImage, cv::Mat& outputImage);

    // Function to highlight edges in an input/output image using an edge mask
    void highlightEdges(cv::Mat& inputOutputImage, const cv::Mat& edges);
};

}
