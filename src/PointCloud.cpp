#include "libobsensor/ObSensor.hpp"
// #include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#include "Utils.hpp"
#include "../include/funcs.hpp"
#include <Eigen/Dense>
#include <execution>

#include <chrono>

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
void saveRGBFrameToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
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

// Save a vector of colored points to a PLY file
void saveRGBPointsToPly(const std::vector<OBColorPoint> &points, std::string fileName) {
    FILE *fp = fopen(fileName.c_str(), "wb+");
    if (!fp) {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    // Write PLY header
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %lu\n", points.size());
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    // Write point data
    for (const auto &point : points) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", 
                point.x, point.y, point.z, 
                static_cast<int>(point.r), 
                static_cast<int>(point.g), 
                static_cast<int>(point.b));
    }

    fclose(fp);
    std::cout << "File saved successfully: " << fileName << std::endl;
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

// Convert a std::shared_ptr<ob::Frame> to a std::vector<OBColorPoint>
std::vector<OBColorPoint> frameToVector(const std::shared_ptr<ob::Frame> &frame) {
    std::vector<OBColorPoint> points;

    if (!frame) {
        std::cerr << "Invalid frame provided!" << std::endl;
        return points; // Return empty vector
    }

    // Calculate the number of points in the frame
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    if (pointsSize <= 0) {
        std::cerr << "No points found in the frame!" << std::endl;
        return points; // Return empty vector
    }

    // Get the raw point data from the frame
    OBColorPoint *rawPoints = static_cast<OBColorPoint *>(frame->data());
    if (!rawPoints) {
        std::cerr << "Failed to retrieve point data from frame!" << std::endl;
        return points; // Return empty vector
    }

    // Copy the points into the vector
    points.assign(rawPoints, rawPoints + pointsSize);

    return points;
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

            // Convert the frame to a vector of OBColorPoint
            std::vector<OBColorPoint> framePointsVec = frameToVector(frame);

            saveRGBPointsToPly(framePointsVec, "RGBPoints.ply");
            std::cout << "RGBPoints.ply saved successfully!" << std::endl;
        } else {
            std::cerr << "Failed to retrieve a valid color or depth frame!" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing RGBD PointCloud: " << e.what() << std::endl;
    }
}

// Function to process and return an RGBD PointCloud frame
std::shared_ptr<ob::Frame> processRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame() && frameset->colorFrame()) {
            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            // Generate the colored point cloud
            std::cout << "Generating RGBD PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            std::cout << "RGBD PointCloud generated successfully!" << std::endl;

            // Return the processed frame
            return frame;
        } else {
            std::cerr << "Failed to retrieve a valid color or depth frame!" << std::endl;
            return nullptr; // Return nullptr if frames are invalid
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing RGBD PointCloud: " << e.what() << std::endl;
        return nullptr; // Return nullptr in case of an exception
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

// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot) {
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    OBColorPoint *originalPoints = (OBColorPoint *)frame->data();

    // Create a vector to store the transformed points
    std::vector<OBColorPoint> transformedPoints(pointsSize);

    for (int i = 0; i < pointsSize; i++) {
        // Convert the point to homogeneous coordinates
        Eigen::Vector4f point_homogeneous(originalPoints[i].x, originalPoints[i].y, originalPoints[i].z, 1.0f);

        // Apply the transformation
        Eigen::Vector4f transformed_point = T_camera_to_robot * point_homogeneous;

        // Populate the transformed points
        transformedPoints[i].x = transformed_point(0);
        transformedPoints[i].y = transformed_point(1);
        transformedPoints[i].z = transformed_point(2);

        // Copy the color information
        transformedPoints[i].r = originalPoints[i].r;
        transformedPoints[i].g = originalPoints[i].g;
        transformedPoints[i].b = originalPoints[i].b;
    }

    // Return the transformed points
    return transformedPoints;
}

// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud2(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot) {
    // Calculate the number of points in the frame
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    OBColorPoint *originalPoints = (OBColorPoint *)frame->data();

    // Create a vector to store the transformed points
    std::vector<OBColorPoint> transformedPoints(pointsSize);

    // Transform points in parallel using std::transform with parallel execution
    std::transform(
        std::execution::par, 
        originalPoints, originalPoints + pointsSize, transformedPoints.begin(),
        [&T_camera_to_robot](const OBColorPoint &point) {
            // Convert the point to homogeneous coordinates
            Eigen::Vector4f point_homogeneous(point.x, point.y, point.z, 1.0f);

            // Apply the transformation
            Eigen::Vector4f transformed_point = T_camera_to_robot * point_homogeneous;

            // Create a transformed OBColorPoint
            OBColorPoint transformed;
            transformed.x = transformed_point(0);
            transformed.y = transformed_point(1);
            transformed.z = transformed_point(2);

            // Copy the color information
            transformed.r = point.r;
            transformed.g = point.g;
            transformed.b = point.b;

            return transformed;
        }
    );

    // Return the transformed points
    return transformedPoints;
}

// Function to read the transformation matrix from a file
Eigen::Matrix4f readTransformationMatrix(const std::string& filePath) {
    // Initialize an empty Eigen::Matrix4f
    Eigen::Matrix4f T_camera_to_QR = Eigen::Matrix4f::Identity();

    // Open the file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file at " + filePath);
    }

    // Skip the header line
    std::string header;
    std::getline(file, header);
    if (header.find("Mean Transformation Matrix:") == std::string::npos) {
        throw std::runtime_error("Error: Unexpected file format. Header not found.");
    }

    // Read the matrix elements
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(file >> T_camera_to_QR(i, j))) {
                throw std::runtime_error("Error: Invalid matrix format in file.");
            }
        }
    }

    // Close the file
    file.close();

    return T_camera_to_QR;
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


    // Path to the file
    std::string filePath_T_camera_to_QR = "/home/enes/Desktop/ORBBEC/Orbbec_Codes/JPG_taker/build/pose_estimation.txt";
    // Read T_camera_to_QR matrix
    Eigen::Matrix4f T_camera_to_QR = readTransformationMatrix(filePath_T_camera_to_QR);
    // Print the transformation matrix
    std::cout << "Transformation Matrix (T_camera_to_QR):" << std::endl;

    T_camera_to_QR = Eigen::Matrix4f::Identity();
    //T_camera_to_QR(1, 3) = 0.0; // Replace 'translation_x' with the desired translation value along x
    T_camera_to_QR(1, 1) = 0.0f;
    T_camera_to_QR(1, 2) = -1.0f;
    T_camera_to_QR(2, 1) = 1.0f;
    T_camera_to_QR(2, 2) = 0.0f;


    // Print each row of the matrix for better readability
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << T_camera_to_QR(i, j) << "\t";
        }
        std::cout << std::endl; // Newline after each row
    }


    // Define T_QR_to_robot (replace with actual values)
    Eigen::Matrix4f T_QR_to_robot;
    // Example: Fill in the matrix values here
    T_QR_to_robot << 1, 0, 0, 0.0,
                      0, 1, 0, 0.0,
                      0, 0, 1, 0.0,
                      0, 0, 0, 1;

    // Compute T_camera_to_robot
    Eigen::Matrix4f T_camera_to_robot = T_QR_to_robot * T_camera_to_QR;

    while(true) {
        auto frameset = pipeline.waitForFrames(100);
        if(InputUtils::kbhit()) {
            int key = InputUtils::getch();
            // Press the ESC key to exit
            if(key == KEY_ESC) {
                break;
            }
            if(key == 'R' || key == 'r') {
                // Process the RGBD point cloud and get the frame
                auto pointCloudFrame = processRGBDPointCloud(pipeline, pointCloud);
                //processAndSaveRGBDPointCloud(pipeline, pointCloud);
                if (pointCloudFrame) {
                    // The frame can be used for further processing or saving
                    std::cout << "PointCloud frame retrieved successfully!" << std::endl;

                    // Optionally save the frame to a PLY file
                    std::vector<OBColorPoint> pointCloudFrame_points = frameToVector(pointCloudFrame);
                    saveRGBPointsToPly(pointCloudFrame_points, "GeneratedRGBPoints.ply");
            

                    //  // Measure execution time
                    // auto start = std::chrono::high_resolution_clock::now();
                    // // Get the transformed point cloud
                    // auto transformedPoints = getTransformedPointCloud(pointCloudFrame, T_camera_to_robot);
                    // auto end = std::chrono::high_resolution_clock::now();

                    // // Calculate elapsed time
                    // std::chrono::duration<double> elapsed = end - start;
                    // std::cout << "FIRST Execution Time: " << elapsed.count() << " seconds" << std::endl;

                    //FIRST Execution Time: 1.04919 seconds
                    //SECOND Execution Time: 0.153547 seconds

                    auto start2 = std::chrono::high_resolution_clock::now();
                    // Get the transformed point cloud
                    auto transformedPoints2 = getTransformedPointCloud2(pointCloudFrame, T_camera_to_robot);
                    auto end2 = std::chrono::high_resolution_clock::now();

                    // Calculate elapsed time
                    std::chrono::duration<double> elapsed = end2 - start2;
                    std::cout << "SECOND Execution Time: " << elapsed.count() << " seconds" << std::endl;
                    

                    // Save the transformed point cloud
                    saveRGBPointsToPly(transformedPoints2, "TransformedRGBPoints.ply");
                } else { std::cerr << "Failed to generate point cloud frame!" << std::endl;}
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
