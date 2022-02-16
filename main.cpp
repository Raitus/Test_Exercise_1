#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void Print(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string addition = "0"){ // Transform a cloud of points to the ".pcd" file
    if (addition == "0"){
        static char iterator{'0'};
        iterator++;
        addition = iterator;
    }
    std::string fileName{"Files/CloudData_ascii_"};
    fileName += addition;
    fileName += ".pcd";
    pcl::io::savePCDFileASCII(fileName, *cloud);
}

void CloudFilling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    const char binary[] = "Files/car.bin"; //Read data from binary file
    std::ifstream binFile(binary, std::ios::binary | std::ios::in); // Open bin file for reading
    float num;
    pcl::PointXYZ point;
    for (int i = 1; binFile.read(reinterpret_cast<char*>(&num), sizeof(num)); ++i) { // Transform binary data to a points of cloud
        if (i == 1) {
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

void Heapify(pcl::PointCloud<pcl::PointXYZ>::Ptr& testPointsCloud, int n, int i, const char symbol) {
    // Find largest among root, left child and right child
    int largest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (symbol == 'x'){ //Define the coordinate to sort
        if (left < n && testPointsCloud->points[left].x > testPointsCloud->points[largest].x)
            largest = left;
        if (right < n && testPointsCloud->points[right].x > testPointsCloud->points[largest].x)
            largest = right;
    }else if (symbol == 'y'){
        if (left < n && testPointsCloud->points[left].y > testPointsCloud->points[largest].y)
            largest = left;
        if (right < n && testPointsCloud->points[right].y > testPointsCloud->points[largest].y)
            largest = right;
    }else{
        if (left < n && testPointsCloud->points[left].z > testPointsCloud->points[largest].z)
            largest = left;
        if (right < n && testPointsCloud->points[right].z > testPointsCloud->points[largest].z)
            largest = right;
    }

    if (largest != i) { // Swap and continue heapifying if root is not largest
        std::swap(testPointsCloud->points[i], testPointsCloud->points[largest]);
        Heapify(testPointsCloud, n, largest, symbol);
    }
}

void HeapSort(pcl::PointCloud<pcl::PointXYZ>::Ptr& testPointsCloud, const char symbol) {
    auto n{static_cast<int>(testPointsCloud->width)};
    for (int i = n / 2 - 1; i >= 0; i--) // Build max heap
        Heapify(testPointsCloud, n, i, symbol);
    for (int i = n - 1; i >= 0; i--) { // Heap sort
        std::swap(testPointsCloud->points[0], testPointsCloud->points[i]);
        Heapify(testPointsCloud, i, 0, symbol); // Heapify root element to get highest element at root again
    }
}

void CloudCenterFind(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                     pcl::PointXYZ& averageData) { // Cloud center by points' mass define for 3 coordinates of cloud
    HeapSort(sourceCloud, 'x');
    for (int i = 0; i < 1; ++i) {
        averageData.x += sourceCloud->points[(sourceCloud->width / 2) + i].x;
    }
    HeapSort(sourceCloud, 'y');
    for (int i = 0; i < 1; ++i) {
        averageData.y += sourceCloud->points[(sourceCloud->width / 2) + i].y;
    }
    HeapSort(sourceCloud, 'z');
    for (int i = 0; i < 1; ++i) {
        averageData.z += sourceCloud->points[(sourceCloud->width / 2) + i].z;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointXYZ averageData;
    CloudCenterFind(cloud, averageData);
    for (auto& point : *cloud) { // Changing all points of the cloud to make the center of it in (0, 0, 0) coordinates
        point.x += averageData.x;
        point.y += averageData.y;
        point.z += averageData.z;
    }
    Print(cloud, "Final"); //Print final result into the .pcd file
    return cloud;
}

void CloudVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /*pcl::visualization::PCLVisualizer viewer; //Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "Cloud of points");
    viewer.saveScreenshot("Files/screenshot.jpg");*/
}

int main() {
    // Define a point cloud with smart pointer to use it in functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    CloudFilling(pointsCloud);
    Print(pointsCloud);
    CloudVisualization(CloudUpdate(pointsCloud));
    return 0;
}