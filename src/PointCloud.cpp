#include "libobsensor/ObSensor.hpp"
// #include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#include "Utils.hpp"
#include "../include/funcs.hpp"


#define KEY_ESC 27
#define KEY_R 82
#define KEY_r 114

// Save point cloud data to ply
void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// Save colored point cloud data to ply
void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    OBColorPoint *point = (OBColorPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// Function to configure and enable the color stream
std::shared_ptr<ob::VideoStreamProfile> configureColorStream(ob::Pipeline &pipeline, std::shared_ptr<ob::Config> config) {
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // Get all stream profiles of the color camera
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            // Select the default profile
            auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
            colorProfile = profile->as<ob::VideoStreamProfile>();
            config->enableStream(colorProfile);
            std::cout << "Color stream enabled successfully." << std::endl;
        } else {
            std::cerr << "No color profiles available for the current device." << std::endl;
        }
    } catch (const ob::Error &e) {
        config->setAlignMode(ALIGN_DISABLE); // Disable alignment if color stream is unavailable
        std::cerr << "Failed to enable color stream: " << e.getMessage() << std::endl;
    }

    if (colorProfile) {
        config->setAlignMode(ALIGN_D2C_HW_MODE); // Enable D2C alignment if color stream is available
    } else {
        config->setAlignMode(ALIGN_DISABLE);
    }

    return colorProfile;
}


// Function to process and save an RGBD PointCloud to a PLY file
void processAndSaveRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame() && frameset->colorFrame()) {
            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            // Generate and save the colored point cloud
            std::cout << "Generating RGBD PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            saveRGBPointsToPly(frame, "RGBPoints.ply");
            std::cout << "RGBPoints.ply saved successfully!" << std::endl;
        } else {
            std::cerr << "Failed to retrieve a valid color or depth frame!" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing RGBD PointCloud: " << e.what() << std::endl;
    }
}

// Function to process and save a Depth PointCloud to a PLY file
void processAndSaveDepthPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame()) {
            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            // Generate and save the point cloud
            std::cout << "Generating Depth PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            savePointsToPly(frame, "DepthPoints.ply");
            std::cout << "DepthPoints.ply saved successfully!" << std::endl;
        } else {
            std::cerr << "Failed to retrieve a valid depth frame!" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing Depth PointCloud: " << e.what() << std::endl;
    }
}

void intro_prompt() {
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "This sample demonstrates how to create a point cloud from the depth and color streams of a device." << std::endl;
    std::cout << "The sample will create a point cloud and save it to a PLY file." << std::endl;
    // operation prompt
    std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
    std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
    std::cout << "Press ESC to exit! " << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
}

// Function to configure and retrieve depth stream profiles
std::shared_ptr<ob::StreamProfileList> configureDepthStream(ob::Pipeline &pipeline, 
                                                            std::shared_ptr<ob::VideoStreamProfile> colorProfile, 
                                                            OBAlignMode &alignMode,
                                                            std::shared_ptr<ob::Config> config) {
    std::shared_ptr<ob::StreamProfileList> depthProfileList;

    try {
        if (colorProfile) {
            // Try hardware-based Depth-to-Color (D2C) alignment profiles
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList && depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
                std::cout << "Hardware-based D2C alignment enabled." << std::endl;
            }

            // Try software-based Depth-to-Color (D2C) alignment profiles
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
            if (depthProfileList && depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_SW_MODE;
                std::cout << "Software-based D2C alignment enabled." << std::endl;
            }

            std::cerr << "No compatible D2C alignment profiles found. Disabling alignment." << std::endl;
        }else {
            depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
        }

        // Fallback to default depth profiles if no color alignment is required or supported
        if (depthProfileList && depthProfileList->count() > 0) {
            
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try {
                // Select the profile with the same frame rate as color.
                if(colorProfile) {
                    depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
                }
            }
            catch(...) {
                depthProfile = nullptr;
            }
            if(!depthProfile) {
                // If no matching profile is found, select the default profile.
                depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
            }
            config->enableStream(depthProfile);

        } else {
            std::cerr << "No depth profiles available for the current device!" << std::endl;
        }

    } catch (const ob::Error &e) {
        std::cerr << "Error configuring depth stream: " << e.getMessage() << std::endl;
        alignMode = ALIGN_DISABLE;
    }

    config->setAlignMode(alignMode);

    if (depthProfileList && depthProfileList->count() > 0) {
        std::cout << "Depth stream successfully configured." << std::endl;
    } else {
        std::cerr << "Failed to configure depth stream." << std::endl;
    }

    return depthProfileList;
}


int main(int argc, char **argv) try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    // create pipeline
    ob::Pipeline pipeline;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = configureColorStream(pipeline, config);

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode                            alignMode = ALIGN_DISABLE;
    
    // Configure depth stream with or without color alignment
    depthProfileList = configureDepthStream(pipeline, colorProfile, alignMode, config);

    // start pipeline with config
    pipeline.start(config);

    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
    auto cameraParam = pipeline.getCameraParam();
    pointCloud.setCameraParam(cameraParam);

    // Display the operation prompt
    intro_prompt();

    while(true) {
        auto frameset = pipeline.waitForFrames(100);
        if(InputUtils::kbhit()) {
            int key = InputUtils::getch();
            // Press the ESC key to exit
            if(key == KEY_ESC) {
                break;
            }
            if(key == 'R' || key == 'r') {
                processAndSaveRGBDPointCloud(pipeline, pointCloud);
            }
            else if(key == 'D' || key == 'd') {
                processAndSaveDepthPointCloud(pipeline, pointCloud);
            }
        }
    }
    // stop the pipeline
    pipeline.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
