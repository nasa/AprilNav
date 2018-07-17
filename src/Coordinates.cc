
using namespace std;
#include <iostream>
#include <fstream>
#include <string>
#include "Coordinates.h"
#include <sstream>
#include <cmath>
#include <vector>
#include <array>


namespace AprilTags{
    //Read tag coordinates from a CSV file. AprilNav won't run without a Coordinates.csv file present
    void Coordinates::readTagLocation(){
        //if(m_coordinates){
            ifstream data("AprilNav/Coordinates.csv");
            if(!data.is_open()) std::cout << "\nERROR: Can't find file with tag coordinates (Coordinates.csv)! \n" << '\n';
            if(!data.is_open()){    //abort if can't find file
                abort();
            }
            
            while(data.good()){
                for (int i = 0; i < 30; i++){
                    getline(data, x_loc, ',');
                    getline(data, y_loc, '\n');
                    
                    double x;
                    istringstream convertx(x_loc);
                    if ( !(convertx >> x) )
                        x = 0;
                    double y;
                    istringstream converty(y_loc);
                    if ( !(converty >> y) )
                        y = 0;
                    
                    coords.push_back({x,y});
                }
            }
            data.close();
        }
}
