#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;

/* TODO: Define a Map class
   Inside the map class, define the mapWidth, mapHeight and grid as a 2D vector
*/
class Map
{
public:
    // Grid dimension
    int mapWidth = 6;
    int mapHeight = 5;
    
    // Grid
    vector<vector<int>> grid
    {
        {0, 1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 0},
    };
};

/* TODO: Define a Planner class
   Inside the Planner class, define the start, goal, cost, movements, and movements_arrows
   Note: The goal should be defined it terms of the mapWidth and mapHeight
*/
class Planner: Map
{
public:
    // Start position
    vector<int> start = {0, 0};
    
    // Goal position
    vector<int> goal = {mapHeight-1, mapWidth-1};
    
    // Cost of movement
    int cost = 1;
    
    // Movement direction
    vector<vector<int>> movements
    {
        {-1, 0}, // Up
        {0, -1}, // Left
        {1, 0},  // Down
        {0, 1}   // Right
    };
    
    // Movement arrows
    vector<char> movements_arrows = {'^', '<', 'v', '>'};
};

/* TODO: Define a print2DVector function which will print 2D vectors of any data type
   Example
   
   Input: 
   vector<vector<int> > a{{ 1, 0 },{ 0, 1 }};
   print2DVector(a);
   vector<vector<string> > b{{ "a", "b" },{ "c", "d" }};
   print2DVector(b);
   
   Output:
   1 0
   0 1
   a b
   c d
   Hint: You need to use templates
*/
template <typename T> void print2DVector(vector<vector<T>> vect)
{
    for (int row = 0; row < vect.size(); row++)
    {
        for (int col = 0; col < vect[row].size(); col++)
        {
            cout << vect[row][col] << " ";
        }
        cout << "\n";
    }
}

/*############ Don't modify the main function############*/
int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Print classes variables
    cout << "Map:" << endl;
    print2DVector(map.grid);
    cout << "Start: " << planner.start[0] << " , " << planner.start[1] << endl;
    cout << "Goal: " << planner.goal[0] << " , " << planner.goal[1] << endl;
    cout << "Cost: " << planner.cost << endl;
    cout << "Robot Movements: " << planner.movements_arrows[0] << " , " << planner.movements_arrows[1] << " , " << planner.movements_arrows[2] << " , " << planner.movements_arrows[3] << endl;
    cout << "Delta:" << endl;
    print2DVector(planner.movements);

    return 0;
}
