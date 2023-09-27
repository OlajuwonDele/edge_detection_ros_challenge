#include <edge_detection/EdgeDetector.hpp>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <dirent.h>

namespace edge_detection {

void EdgeDetector::detectEdges(const cv::Mat& inputImage, cv::Mat& outputImage) {
    // Convert the input image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(inputImage, grayImage, cv::COLOR_BGR2GRAY);

    // Apply bilateral filtering for denoising while preserving edges
    cv::Mat denoisedImage;
    cv::bilateralFilter(grayImage, denoisedImage, 10, 75, 75);

    // Apply the Canny edge detector
    cv::Mat cannyEdges;
    cv::Canny(denoisedImage, cannyEdges, 40, 200); // You can adjust the threshold values as needed

    // Convert the binary edges to 3-channel format to match the input image
    cv::cvtColor(cannyEdges, outputImage, cv::COLOR_GRAY2BGR);

    // Ensure the dimensions are the same as the input image
    if (outputImage.size() != inputImage.size()) {
        cv::resize(outputImage, outputImage, inputImage.size());
    }
}




void EdgeDetector::highlightEdges(cv::Mat& inputImage, const cv::Mat& edges) {
    for (int i = 0; i < inputImage.rows; ++i) {
        for (int j = 0; j < inputImage.cols; ++j) {
            if (edges.at<uchar>(i, j) > 0) {
                // Highlights the computed edges in green
                inputImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
            }
        }
    }
}

#ifdef EDGE_DETECTOR_TEST // Define this macro only when testing in EdgeDetector.cpp

int main() {
    std::string data_directory = "~/Downloads/data"; // Replace with the actual directory path

    // Expand the tilde (~) character in the directory path
    const char* home_directory = getenv("HOME");
    if (home_directory) {
        data_directory.replace(0, 1, home_directory);
    } else {
        std::cerr << "Error: Could not get the home directory." << std::endl;
        return 1;
    }

    // Open the directory
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(data_directory.c_str())) != nullptr) {
        // Iterate through files in the directory
        while ((ent = readdir(dir)) != nullptr) {
            std::string filename = ent->d_name;
            if (filename.find(".jpg") != std::string::npos || filename.find(".png") != std::string::npos) {
                // Only process files with .jpg or .png extensions (you can add more extensions as needed)

                // Construct the full path to the image file
                std::string imagePath = data_directory + "/" + filename;

                // Load the input image
                cv::Mat inputImage = cv::imread(imagePath);

                if (inputImage.empty()) {
                    std::cerr << "Error: Unable to load input image " << imagePath << std::endl;
                } else {
                    EdgeDetector detector;
                    cv::Mat edges(inputImage.rows, inputImage.cols, inputImage.type());
                    detector.detectEdges(inputImage, edges);
                    cv::Mat outputImage = inputImage.clone();
                    detector.highlightEdges(outputImage, edges);

                    cv::imshow("Edge Detection Result", edges);
                    cv::imshow("Edges Highlighted", outputImage);
                    cv::waitKey(0);

                }
            }
        }
        closedir(dir);
    } else {
        // Could not open the directory, provide error details
        std::cerr << "Error: Could not open data directory " << data_directory << std::endl;
        std::cerr << "Error details: " << strerror(errno) << std::endl;
    }

    return 0;
}

#endif // EDGE_DETECTOR_TEST

} // namespace edge_detection
