#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <vector>
#include <string>
#include <unistd.h>
#include <dirent.h>

int main(int argc, char* argv[]){

    float indPrecision;
    std::string value;
    std::vector<float> avgPrecision;
    // Parse through all the Text Files and Compute Average Precision
    struct dirent *pDirent;
    DIR *pDir = opendir("/home/buzkent/Downloads/Results/");
    int fileFirst = 0;
    while ((pDirent = readdir(pDir)) != NULL) {

       std::string fileName = pDirent->d_name;
       std::string homeDir = "/home/buzkent/Downloads/Results/";
       std::string fullDir = homeDir.append(fileName);
       std::ifstream prFile(fullDir);
       std::string toAvoid = ".";
       if (fileName.compare(toAvoid) == 0 || fileName.compare("..") == 0 || fileName.compare("AveragePrecision") == 0){
          continue;
       }
       if (!prFile)
       {
         return 0;
       }
       int index = 0;
       char ch;
       while (prFile >> indPrecision){
	   if (fileFirst == 0){
              avgPrecision.push_back(indPrecision);
           }
           else{
              avgPrecision[index] += indPrecision;
              index++;
           }
       }
       fileFirst++;
       
    }
    closedir(pDir);

    // Save the Results into Another Text File
    for (int i = 0; i < avgPrecision.size() ; i++){
        avgPrecision[i] /= fileFirst;
    }
    std::ofstream output_file("/home/buzkent/Downloads/Results/AveragePrecision/Precision.txt");
    std::ostream_iterator<float> output_iterator(output_file, "\n");
    std::copy(avgPrecision.begin(), avgPrecision.end(), output_iterator);
  
    return 0;
}
