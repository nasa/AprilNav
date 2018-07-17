using namespace std;
#include <stdio.h>
#include <string>
#include <vector>
#include <array>

namespace AprilTags{
  class Coordinates {
  public:
    vector <array<double,2> > coords; //Array of Coordinates
    void readTagLocation();
  private:
    string x_loc;   //can't use double or won't recognize 'getline'
    string y_loc;

  };
}
