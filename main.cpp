#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void Print(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    static char iterator{'0'};
    iterator++;
    std::string fileName{"Files/CloudData_ascii_"};
    fileName += iterator;
    fileName += ".pcd";
    pcl::io::savePCDFileASCII(fileName, *cloud);
}

void CloudFilling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    const char binary[] = "Files/car.bin";
    std::ifstream binFile(binary, std::ios::binary | std::ios::in);
    float num;
    pcl::PointXYZ point;
    for (int i = 1; binFile.read(reinterpret_cast<char*>(&num), sizeof(num)); ++i) {
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
    binFile.close();
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

void MassCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                pcl::PointXYZ& maxElements,
                pcl::PointXYZ& minElements,
                float& xAverage,
                float& yAverage,
                float& zAverage){
    pcl::PointCloud<pcl::PointXYZ>::Ptr testPointsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    testPointsCloud = cloud;

    HeapSort(testPointsCloud, 'x');
    Print(testPointsCloud);
    for (int i = 0; i < 1; ++i) {
        xAverage += testPointsCloud->points[(testPointsCloud->width / 2) + i].x;
    }
    std::cout<<"xAverage: "<<xAverage<<std::endl;
    HeapSort(testPointsCloud, 'y');
    Print(testPointsCloud);
    for (int i = 0; i < 1; ++i) {
        yAverage += testPointsCloud->points[(testPointsCloud->width / 2) + i].y;
    }
    std::cout<<"yAverage: "<<yAverage<<std::endl;
    HeapSort(testPointsCloud, 'z');
    Print(testPointsCloud);
    for (int i = 0; i < 1; ++i) {
        zAverage += testPointsCloud->points[(testPointsCloud->width / 2) + i].z;
    }
    std::cout<<"zAverage: "<<zAverage<<std::endl;
}

void CloudCenterFind(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                     float& xAverage,
                     float& yAverage,
                     float& zAverage) {
    pcl::PointXYZ maxElements{*sourceCloud->begin()}, minElements{*sourceCloud->begin()};
    for (auto& point: *sourceCloud) {
        if (point.x > maxElements.x) {
            maxElements.x = point.x;
        } else if (point.x < minElements.x) {
            minElements.x = point.x;
        }
        if (point.y > maxElements.y) {
            maxElements.y = point.y;
        } else if (point.y < minElements.y) {
            minElements.y = point.y;
        }
        if (point.z > maxElements.z) {
            maxElements.z = point.z;
        } else if (point.z < minElements.z) {
            minElements.z = point.z;
        }
    }
    xAverage = (maxElements.x + minElements.x) / 2;
    yAverage = (maxElements.y + minElements.y) / 2;
    zAverage = (maxElements.z + minElements.z) / 2;
    MassCenter(sourceCloud, maxElements, minElements, xAverage, yAverage, zAverage);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float xAverage, yAverage, zAverage;
    CloudCenterFind(cloud, xAverage, yAverage, zAverage);
    for (auto& point : *cloud) {
        point.x += xAverage;
        point.y += yAverage;
        point.z += zAverage;
    }
    return cloud;
}

void CloudVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    /*pcl::visualization::PCLVisualizer viewer; //Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "Cloud of points");
    viewer.saveScreenshot("Files/screenshot.jpg");*/
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    CloudFilling(pointsCloud);
    Print(pointsCloud);

    CloudVisualization(CloudUpdate(pointsCloud));

    return 0;
}