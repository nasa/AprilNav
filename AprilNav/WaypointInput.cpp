/* Accepts input waypoint input from a user. Input.cc parses the Input
 * to determine if a valid coordinate or tag id was entered
 */

#include <string>
#include <iostream>
#include <thread>
#include <cstring>
#include <fstream>

using namespace std;

ofstream input;

int main(){

  input.open("input.txt");
  input << "INPUT" << endl;
  string WP;

  while(true){
    cout << "Enter new Waypoint:" << endl;
    cin >> WP;
    input << WP << endl;
  }
}
