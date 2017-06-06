#!/bin/sh

i=1

# UAV123
cd /home/buzkent/Downloads/UAV123_10fps/data_seq/UAV123_10fps/

# VOT15
# cd /home/buzkent/Downloads/VOT15/

for item in *

   do 

    # echo "Item : $item"
    # UAV123
    /home/buzkent/paper/C++_Implementation/build/EnKCF -e image -d /home/buzkent/Downloads/UAV123_10fps/data_seq/UAV123_10fps/"$item"/ -g /home/buzkent/Downloads/UAV123_10fps/anno/UAV123_10fps/"$item".txt -p "$item"
    # VOT15
    # /home/buzkent/paper/C++_Implementation/build/EnKCF -e image -d /home/buzkent/Downloads/VOT15/"$item"/ -g /home/buzkent/Downloads/VOT15/"$item"/groundtruth.txt -p "$item"
   done
