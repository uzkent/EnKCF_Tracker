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

    float indPrecision, indSuccessOverlap;
    std::vector<std::vector<float> > avgPrecision(2);
    std::ofstream output_file("/home/buzkent/Downloads/Results/AveragePrecision/Precision.txt");

    // Parse through all the Text Files and Compute Average Precision
    struct dirent *pDirent;
    DIR *pDir = opendir("/home/buzkent/Downloads/Results/");
    int fileFirst = 0;
    char ch;
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
       while (prFile >> indPrecision >> ch >> indSuccessOverlap){
	   if (fileFirst == 0){
              avgPrecision[0].push_back(indPrecision);
              avgPrecision[1].push_back(indSuccessOverlap);
           }
           else{
              avgPrecision[0][index] += indPrecision;
              avgPrecision[1][index] += indSuccessOverlap;
              index++;
           }
       }
       fileFirst++;
       
    }
    closedir(pDir);

    // Save the Results into Another Text File
    for (int i = 0; i < avgPrecision[0].size() ; i++){
        avgPrecision[0][i] /= fileFirst;
        avgPrecision[1][i] /= fileFirst;
        output_file << avgPrecision[0][i] << "," << avgPrecision[1][i];
        output_file << std::endl;       
    }

    // Compute the Area Under Curve for Precision and Success Overlap
    float aucPrecision = 0;
    float aucSuccess = 0;
    for (int i = 0; i < avgPrecision[0].size() ; i++){
        if ( i <= 50 ){
            std::cout << avgPrecision[0][i] << std::endl;
            aucPrecision += avgPrecision[0][i];
        }
        aucSuccess += avgPrecision[1][i];
    }    
    output_file << aucPrecision * 2 << "," << aucSuccess << std::endl;
       
    return 0;
}
