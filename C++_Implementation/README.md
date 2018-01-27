## Description
This is the *C++ implementation* of the proposed `EnKCF tracker`. It includes implementation of a *bootsrap particle filter* and *ensemble of kernelized correlation filters*. We suggest the user to disable the particle filter in the case of uncompensated platfrom motion. You can find the information to compile and run the tracker below.

### To Compile
```Shell
cd C++_Implementation
mkdir build
cd build
cmake ..
```

### To Run for Video Input
```Shell
   ./EnKCF -d video -e "Video of Interest" -g "Ground Truth" -p "Text File for Precision and Success Curves"
```

### To Run for Image Series Input
```Shell
   ./EnKCF -d image -e "Images Folder" -g "Ground Truth" -p "Text File for Precision and Success Curves"
```

### To Run the Tracker on Benchmark Datasets

Our EnKCF tracking C++ code currently supports the following datasets.

1. [UAV123 Dataset](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/RunTracking/UAV123.sh)
2. [UAV20L Dataset](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/RunTracking/UAV20L.sh)
3. [OTB100 Dataset](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/RunTracking/OTB100.sh)
4. [UAV123_10fps Dataset](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/RunTracking/UAV123_10fps.sh)

You need to uncomment the desired dataset name in the [cmake file](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/CMakeLists.txt) and provide the right directories in
the corresponding `shell script file` and [main source file](https://github.com/buzkent86/EnKCF_Tracking_WACV18/blob/master/C%2B%2B_Implementation/main/src/runtracker.cpp).
