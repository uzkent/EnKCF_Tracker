#!/bin/sh

i=1

# UAV123
cd /home/buzkent/Downloads/UAV123/data_seq/UAV123/

IFS=" "

while read sFrame lFrame fName vName
do
 
    # UAV123
    /home/buzkent/paper/C++_Implementation/build/EnKCF -e image -d /home/buzkent/Downloads/UAV123/data_seq/UAV123/"$vName"/ -g /home/buzkent/Downloads/UAV123/anno/UAV123/"$fName".txt -p "$fName"
   
done < /home/buzkent/Desktop/startFrames_UAV123.txt
