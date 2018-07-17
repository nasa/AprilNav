/* This code runs when the -i flag is entered with AprilNav. WaypointInput.cpp is is opened
 * in a new terminal window, and an implementation of the tail function follows the last line
 * of the generated Input.txt file to check if new Waypoints have been sent by the user
 */
 
#include "Input.h"
#include <iostream>
#include <sstream>

#ifdef __APPLE__
#define NEWTERMINAL "open -a Terminal -n "
#endif

#ifdef __unix__
#define NEWTERMINAL "xterm -e "
#endif

#ifdef __arm__
#define NEWTERMINAL "xterm -e "
#endif

#define SIZE 100

char oldwp[255];  //for checking if there is a new waypoint

using namespace std;

namespace AprilTags{
  double Input::getX(){
    return WPX;
  }
  double Input::getY(){
    return WPY;
  }

  void Input::inputParse(string str){
    vect.clear();
    std::stringstream ss(str);
    double i;

    while (ss >> i)
    {
      vect.push_back(i);

      if (ss.peek() == ',')
      ss.ignore();
    }

    if (vect.size() == 2){
      cout <<
      "X: " << vect.at(0) << "  " <<
      "Y: " << vect.at(1) << endl;
      WPX = vect.at(0);
      WPY = vect.at(1);
    }

    else if (vect.size() == 1){
      verifyTag(vect.at(0));
      //cout << "TagID: " << vect.at(0) << endl;
    }

    else{
      //cout << "\033[32mINVALID INPUT.\032[m" << endl;
      cout << "\033[0;33mINVALID INPUT.\033[0m" << endl;
    }
  }

//Variation of https://www.geeksforgeeks.org/implement-your-own-tail-read-last-n-lines-of-a-huge-file/
  void Input::tail(int n)
  {
    int count = 0;  // To count '\n' characters

    // unsigned long long pos (stores upto 2^64 – 1
    // chars) assuming that long long int takes 8
    // bytes
    unsigned long long pos;
    char wp[2*SIZE];

    // Go to End of file
    if (fseek(fp, 0, SEEK_END))
    perror("fseek() failed");
    else
    {
      // pos will contain no. of chars in
      // input file.
      pos = ftell(fp);

      // search for '\n' characters
      while (pos)
      {
        // Move 'pos' away from end of file.
        if (!fseek(fp, --pos, SEEK_SET))
        {
          if (fgetc(fp) == '\n')

          // stop reading when n newlines
          // is found
          if (count++ == n)
          break;
        }
        else
        perror("fseek() failed");
      }

      // print last n lines
      while (fgets(wp, sizeof(wp), fp)){
        if (wp != oldwp){
          inputParse(wp);
        }
        //check if there is new input
        for(int i = 0 ; i < sizeof(wp) ; i++)
        oldwp[i] = wp[i];
      }
    }
  }

//Check Tag valitidy of entered ID to see if it exists
  void Input::verifyTag(int id){
    if (id > coords.size() || id < 0)
      cout << "\033[1;31mInvalid Tag Location, Does not exist.\033[0m" << endl;
    else{
      WPX = coords.at(id)[0];
      WPY = coords.at(id)[1];
      cout << "TagID: " << id << "  " <<
      "X: " << WPX << "  " <<
      "Y: " << WPY << endl;
    }
  }

  // Opens the input file for autonomous waypoints
  void Input::setup(FILE* FP, vector <array<double,2> > COORDS){
    //Copy Coordinates from AprilNav to verify id validity
    copy(COORDS.begin(),COORDS.end(),back_inserter(coords));
    fp = FP;

    //Open WayPointInput in a second terminal window to accept user input Waypoints
    string command = "'./build/bin/WaypointInput' &";
    string term = NEWTERMINAL + command;
    system(term.c_str());

    char buffer[SIZE];

    // Open file in binary mode
    // wb+ mode for reading and writing simultaneously
    fp = fopen("input.txt", "wb+");
    if (fp == NULL)
    {
      printf("Error while opening file");
      exit(EXIT_FAILURE);
    }
  }
}
