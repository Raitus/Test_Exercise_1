#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

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
    viewer.setBackgroundColor (0, 0, 0);   // I suppose that it is because of VTK version (8.2) and its combine
    viewer.addCoordinateSystem (1.0, 0, 0, 0); // with PCL 1.12, if we will see on previous reports at
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "Cloud of points"); // PCL library report tickets and changed PCL
    viewer.initCameraParameters();                                     // version at 1.11.1.
    viewer.setCameraPosition(0, 20, 10,    0, 0, 0,   0, 0, 0);
    viewer.setCameraFieldOfView(0.523599);
    viewer.saveScreenshot("Files/screenshot.jpg");
    viewer.close();
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