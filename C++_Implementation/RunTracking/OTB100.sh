#!/bin/sh

i=1

# OTB100
cd /home/buzkent/Downloads/OTB100/

for item in *

   do 

    # echo "Item : $item"
    # OTB100
    /home/buzkent/paper/C++_Implementation/build/EnKCF -e image -d /home/buzkent/Downloads/OTB100/"$item"/img/ -g /home/buzkent/Downloads/OTB100/"$item"/groundtruth_rect.txt -p "$item"
   done
