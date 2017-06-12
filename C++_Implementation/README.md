## Description

This is the C++ implementation of the proposed EnKCF tracker. It includes implementation of a bootsrap particle filter and ensemble of kernelized correlation filters. We suggest the user to disable the particle filter in the case of uncompensated platfrom motion. You can find the information to compile and run the tracker below.

### To Compile
<ul> 
<li> cd C++_Implementation
<li> mkdir build
<li> cd build
<li> cmake ..
</ul>

### To Run for Video Input
<ul>
<li> ./EnKCF -d video -e "Video of Interest" -g "Ground Truth" -p "Text File for Precision and Success Curves"
</ul>

### To Run for Series of Images Input
<ul>
<li> ./EnKCF -d image -e "Images Folder" -g "Ground Truth" -p "Text File for Precision and Success Curves"
</ul>
