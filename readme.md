**Test task C++**


**Problem**

You need to provide a console application that can work with points in 3D space. This app should be able to solve following tasks:

* Find center of given set of points(point cloud)

* Transform set of points to match center of coordinates with centr of point cloud

* Save visualization of point clouds as jpg images


**Application input**

As an input application should get a path to the binary file (car.bin, for example).

The following is a pseudocode illustrating the method of dumping (serialization) of a point cloud (vector of 3D point) to the car.bin file:


    std::ofstream binary("filename.bin", std::ios::out | std::ios::binary | std::ios::trunc);
    for (const auto &point : points){
        binary.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
        binary.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
        binary.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
    }


**Application output**

Picture of point cloud saved in binary file in JPG format.


**Deliverables**
* Compilable code with comments, that can be useful in understanding your method.
* Executable file of this code.
* 2 pictures in JPG/JPEG/PNG format in different points of view that can represent content in a binary file (feel free to choose point of view by yourself).


**Additional information**
* You can use any side-libraries, but PCL or OpenCV is preferable.
* You should use CMake as configuration system
* C++ standard shouldn’t be lower than 17-th 
