#!/bin/sh

i=1

# UAV123
cd /home/buzkent/Downloads/UAV123_10fps/data_seq/UAV123_10fps/

IFS=" "

while read sFrame lFrame fName vName
do
 
    # UAV123
    /home/buzkent/paper/C++_Implementation/build/EnKCF -e image -d /home/buzkent/Downloads/UAV123_10fps/data_seq/UAV123_10fps/"$vName"/ -g /home/buzkent/Downloads/UAV123_10fps/anno/UAV123_10fps/"$fName".txt -p "$fName"
   
done < /home/buzkent/Desktop/startFrames_UAV123_10fps.txt
