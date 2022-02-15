#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

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

void
CloudCenterFind(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud, float& xAverage, float& yAverage, float& zAverage) {
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
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud) {
    float xAverage, yAverage, zAverage;
    CloudCenterFind(sourceCloud, xAverage, yAverage, zAverage);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for ( int i{0}; i != sourceCloud->size(); i++) {
        newCloud->begin()+i = sourceCloud->begin()+i->x += xAverage;
        point.y += yAverage;
        point.z += zAverage;
    }
    pcl::io::savePCDFileBinary("Files/newCloudData_bin.pcd", *newCloud);
    pcl::io::savePCDFileASCII("Files/newCloudData_ascii.pcd", *newCloud);
    return newCloud;
}

void CloudVisualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    /*pcl::visualization::PCLVisualizer viewer; //Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "Cloud of points");
    viewer.saveScreenshot("Files/screenshot.jpg");*/
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    CloudFilling(pointsCloud);
    pcl::io::savePCDFileBinary("Files/cloudData.pcd", *pointsCloud);

    CloudVisualization(CloudUpdate(pointsCloud));

    return 0;
}