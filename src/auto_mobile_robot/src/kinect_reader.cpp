#include <iostream>
#include <libfreenect/libfreenect.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>  // ✅ Added mutex

class KinectDevice : public Freenect::FreenectDevice {
private:
    std::vector<uint8_t> depthBuffer;
    std::vector<uint8_t> rgbBuffer;
    cv::Mat depthMat;
    cv::Mat rgbMat;
    bool newDepthFrame;
    bool newRGBFrame;
    std::mutex mtx;  // ✅ Added mutex

public:
    KinectDevice(freenect_context *ctx, int index)
        : Freenect::FreenectDevice(ctx, index),
          depthBuffer(640 * 480 * 4), rgbBuffer(640 * 480 * 3),
          depthMat(480, 640, CV_8UC1), rgbMat(480, 640, CV_8UC3),
          newDepthFrame(false), newRGBFrame(false) {}

    void VideoCallback(void *rgb, uint32_t timestamp) override {
        std::lock_guard<std::mutex> lock(mtx);  // ✅ Use mtx instead of mutex
        memcpy(rgbBuffer.data(), rgb, 640 * 480 * 3); // ✅ Safe buffer copy
        newRGBFrame = true;
    }

    void DepthCallback(void *depth, uint32_t timestamp) override {
        std::lock_guard<std::mutex> lock(mtx);
        uint16_t *depthData = static_cast<uint16_t *>(depth);
        for (int i = 0; i < 640 * 480; i++) {
            depthMat.data[i] = depthData[i] >> 4; // Normalize to 8-bit
        }
        newDepthFrame = true;
    }

    bool getRGB(cv::Mat &output) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!newRGBFrame)
            return false;
        rgbMat = cv::Mat(480, 640, CV_8UC3, rgbBuffer.data()).clone(); // ✅ Clone buffer to prevent corruption
        cv::cvtColor(rgbMat, output, cv::COLOR_RGB2BGR);
        newRGBFrame = false;
        return true;
    }

    bool getDepth(cv::Mat &output) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!newDepthFrame)
            return false;
        depthMat.copyTo(output);
        newDepthFrame = false;
        return true;
    }
};

int main() {
    Freenect::Freenect freenect;
    KinectDevice &device = freenect.createDevice<KinectDevice>(0);
    device.startVideo();
    device.startDepth();

    cv::Mat rgbFrame, depthFrame;
    while (true) {
        if (device.getRGB(rgbFrame)) {
            cv::imshow("RGB", rgbFrame);
        }
        if (device.getDepth(depthFrame)) {
            cv::imshow("Depth", depthFrame);
        }
        if (cv::waitKey(30) == 27)
            break; // Press 'ESC' to exit
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}
