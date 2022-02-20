#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include </home/raitus/ProgrammingLibraries/CImg/CImg/CImg.h>

/*void Print(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string addition = "0"){ // Transform a cloud of points
    if (addition == "0"){                                                                 // to a ".pcd" file
        static char iterator{'0'};
        iterator++;
        addition = iterator;
    }
    std::string fileName{"Files/CloudData_ascii_"};
    fileName += addition;
    fileName += ".pcd";
    pcl::io::savePCDFileASCII(fileName, *cloud);
}*/

void CloudFilling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    const char binary[] = "Files/car.bin"; //Read data from binary file
    std::ifstream binFile(binary, std::ios::binary | std::ios::in); // Open bin file for reading
    float num;
    pcl::PointXYZ point;
    for (int i = 1; binFile.read(reinterpret_cast<char*>(&num), sizeof(num)); ++i) { // Transform binary data to
        if (i == 1) {                                                                     // a points of cloud
            point.x = num;
        } else if (i == 2) {
            point.y = num;
        } else {
            point.z = num;
            cloud->push_back(point);
            i = 0;
        }
    }
    binFile.close(); // Close the binary file
}

void CloudCenterFind(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                     pcl::PointXYZ& averageData) { // Cloud center by points' mass define for 3 coordinates of cloud
    // A solution to find the center of the cloud in summarizing all coordinates of one axe and divide a sum on its count
    for (int i = 0; i < sourceCloud->width; ++i) {
        averageData.x += sourceCloud->points[i].x;
    }
    averageData.x/=static_cast<float>(sourceCloud->width);

    for (int i = 0; i < sourceCloud->width; ++i) {
        averageData.y += sourceCloud->points[i].y;
    }
    averageData.y/=static_cast<float>(sourceCloud->width);

    for (int i = 0; i < sourceCloud->width; ++i) {
        averageData.z += sourceCloud->points[i].y;
    }
    averageData.z/=static_cast<float>(sourceCloud->width);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointXYZ averageData;
    CloudCenterFind(cloud, averageData); //Find center of the cloud with identical mass of points
    for (auto& point : *cloud) { // Changing all points of the cloud to make the center of it in (0, 0, 0)
        point.x += averageData.x;          // coordinates
        point.y += averageData.y;
        point.z += averageData.z;
    }
    // Print(cloud, "Final"); //Print final result into the .pcd file
    return cloud;
}

void CloudVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::visualization::PCLVisualizer viewer; // Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
    // I suppose that it is because of VTK version (8.2) and its combine with PCL 1.12, if we will see on previous
    // reports at PCL library report tickets and changed PCL version at 1.11.1.
    // There is I spend a lot of time because of VTK errors, but I decided them by changing PCL library version from 1.12
    // to PCL 1.11.1 and after connecting all necessary modules in CMakeList.txt

    viewer.setBackgroundColor (0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "Cloud of points");
    viewer.initCameraParameters(); // Camera parameters initialisation to choose the point of view
    viewer.setCameraPosition(0, 20, 10,    0, 0, 0,   0, 0, 0);
    viewer.setCameraFieldOfView(0.523599); // Function for zoom the cloud at the image
    viewer.saveScreenshot("Files/screenshot"); // I think that it is terrible solution, but regrettably I don't
    viewer.close();                                // clearly understand alternative solution from PCL documentation
    // I suppose it can be decided using PointCloudGeometryHandler and libjpeg.

    // External library usage for converting ".png" to ".jpg" file format
    cimg_library::CImg<unsigned char> image("Files/screenshot"); // I tried to find a solution to convert .pcd
    image.save_jpeg("Files/result.jpg");                         // data into .jpeg file. Thought about VTK
    image.clear();                                                       // Image writer, Boost GIL (compatible for C++11)
    std::remove("Files/screenshot");                             // But decided to stop on CImg library because
                                                                         // of its portability and compactness
}

int main() {
    // Define a point cloud with smart pointer to use it in functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill the cloud using points from "car.bin" file
    CloudFilling(pointsCloud);
    // Take a screenshot of the cloud using PCLVisualizer
    CloudVisualization(CloudUpdate(pointsCloud));
    //Print(pointsCloud, "After_update"); // Used to look at points after update
    return 0;
}