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
    std::ofstream precisionFile("/home/buzkent/Downloads/Results/AveragePrecision/Precision.txt");
    std::ofstream successFile("/home/buzkent/Downloads/Results/AveragePrecision/Success.txt");
    std::ifstream runTimeFile("/home/buzkent/Downloads/Results/RunTime/RunTimes.txt");

    // Parse through all the Text Files and Compute Average Precision
    struct dirent *pDirent;
    DIR *pDir = opendir("/home/buzkent/Downloads/Results/Precision/");
    int fileFirst = 0;
    char ch;
    while ((pDirent = readdir(pDir)) != NULL) {

       std::string fileName = pDirent->d_name;
       std::string homeDir = "/home/buzkent/Downloads/Results/Precision/";
       std::string fullDir = homeDir.append(fileName);
       std::ifstream prFile(fullDir);
       std::string toAvoid = ".";
       if (fileName.compare(toAvoid) == 0 || fileName.compare("..") == 0){
          continue;
       }
       if (!prFile)
       {
         return 0;
       }
       int index = 0;
       while (prFile >> indPrecision){
	   std::cout << indPrecision << std::endl;
           if (fileFirst == 0){
              avgPrecision[0].push_back(indPrecision);
           }
           else{
              avgPrecision[0][index] += indPrecision;
              index++;
           }
       }
       fileFirst++;
       
    }
    closedir(pDir);

    // Save the Results into Another Text File
    for (int i = 0; i < avgPrecision[0].size() ; i++){
        avgPrecision[0][i] /= fileFirst;
        precisionFile << avgPrecision[0][i];
        precisionFile << std::endl;
    }

    // struct dirent *pDirent; 
    pDir = opendir("/home/buzkent/Downloads/Results/Success/");
    int fileFirst2 = 0;
    while ((pDirent = readdir(pDir)) != NULL) {

       std::string fileName = pDirent->d_name;
       std::string homeDir = "/home/buzkent/Downloads/Results/Success/";
       std::string fullDir = homeDir.append(fileName);
       std::ifstream sucFile(fullDir);
       std::string toAvoid = ".";
       std::cout << fullDir << std::endl;
       if (fileName.compare(toAvoid) == 0 || fileName.compare("..") == 0){
          continue;
       }
       if (!sucFile)
       {
         return 0;
       }
       int index = 0;
       while (sucFile >> indSuccessOverlap){
           if (fileFirst2 == 0){
              avgPrecision[1].push_back(indSuccessOverlap);
           }
           else{
              avgPrecision[1][index] += indSuccessOverlap;
              index++;
           }
       }
       fileFirst2++;
    }
    closedir(pDir);

    // Save the Results into Another Text File
    for (int i = 0; i < avgPrecision[1].size() ; i++){
        avgPrecision[1][i] /= fileFirst2;
        successFile << avgPrecision[1][i];
        successFile << std::endl; 
    }

    // Compute the Area Under Curve for Success Overlap
    float aucSuccess = 0;
    for (int i = 0; i < avgPrecision[1].size() ; i++){
        aucSuccess += avgPrecision[1][i];
    }    
    successFile << aucSuccess << std::endl;
      
    // Compute the Average Run time
    float indRunTime;
    float avgRunTime = 0;
    int counter = 0;
    while( runTimeFile >> indRunTime ){
        avgRunTime += indRunTime;
        counter++;
    }
    successFile << avgRunTime/(float) counter;

    return 0;
}
