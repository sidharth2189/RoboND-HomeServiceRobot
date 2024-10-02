#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;

// Map class
class Map {
public:
    const static int mapWidth = 6;
    const static int mapHeight = 5;
    vector<vector<int> > grid = {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 }
    };
    
    vector<vector<int> > heuristic = {
        { 9, 8, 7, 6, 5, 4 },
        { 8, 7, 6, 5, 4, 3 },
        { 7, 6, 5, 4, 3, 2 },
        { 6, 5, 4, 3, 2, 1 },
        { 5, 4, 3, 2, 1, 0 }
    };
};

// Planner class
class Planner : Map {
public:
    int start[2] = { 0, 0 };
    int goal[2] = { mapHeight - 1, mapWidth - 1 };
    int cost = 1;

    vector<char> movements_arrows = { '^', '<', 'v', '>' };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };
};

// Template function to print 2D vectors of any type
template <typename T>
void print2DVector(T Vec)
{
    for (int i = 0; i < Vec.size(); ++i) {
        for (int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
}

/* #### TODO: Modify the search function and generate the policy vector #### */
void search(Map map, Planner planner)
{
    // Initial open list contains start position
    vector<vector<int>> open{
        {map.heuristic[planner.start[0]][planner.start[1]], 
        0, 
        planner.start[0], 
        planner.start[1]}
    };
    
    // Keep track of traversed cells in order to avoid expanding them again
    vector<vector<int>> traversed{{planner.start[0], planner.start[1]}};

    // Expansion list with initial cell values as -1 and start position as 0
    int expand = 0;
    vector<vector<int>> expansion(map.mapHeight, vector<int> (map.mapWidth, -1));
    
    // Policy list with initial values as "-"
    vector<vector<char>> policy(map.mapHeight, vector<char> (map.mapWidth, '-'));
    policy[planner.goal[0]][planner.goal[1]] = '*';
    
    // Create action array filled with -1
    vector<vector<int> > action(map.mapHeight, vector<int>(map.mapWidth, -1));
     
    while (!open.empty())
    {
        // Sort open list
        sort(open.begin(), open.end());
        
        // Expansion list
        expansion[open[0][2]][open[0][3]] = expand;
        expand++;
        
        // Expand first cell in open. If cell to be expanded is the goal, break.
        if ((open[0][2] == planner.goal[0]) && (open[0][3] == planner.goal[1]))
        {
            // Print path cost and goal
            //cout << open[0][0] << " " << open[0][1] << " " << open[0][2] << endl;
            break;
        }
        
        for (int i = 0; i < planner.movements.size(); i++)
        {        
            // Expand first element in open list
            int g = open[0][1] + planner.cost;
            int x = open[0][2] + planner.movements[i][0];
            int y = open[0][3] + planner.movements[i][1];
            vector<int> loc{x, y};
        
            // If explored cell is within grid, not closed, not already in open list 
            // and not already traversed, then add to open list.
            if ((x < map.mapHeight) && (x >= 0) && (y < map.mapWidth) && (y >= 0) && (map.grid[x][y] != 1) && 
                (find(traversed.begin(), traversed.end(), loc) == traversed.end()))
            {
                int f = g + map.heuristic[x][y];
                vector<int> cell{f, g, x, y};
                if ((find(open.begin(), open.end(), cell) == open.end()))
                {
                    open.push_back(cell);
                    traversed.push_back(loc);
                    action[x][y] = i;
                }
            }
    
        }

        // Remove expanded cell after exploration
        open.erase(open.begin());

        // Roadblock
        if (open.empty())
        {
            cout << "Failed to reach the goal" << endl;
            break;
        }

    }

    // Print cell expansion list
    print2DVector(expansion);
    
    // Print the path with arrows
    // Going backward
    int x = planner.goal[0];
    int y = planner.goal[1];
    int x2, y2;
    policy[x][y] = '*';

    while (x != planner.start[0] or y != planner.start[1]) 
    {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }
    print2DVector(policy);
    //print2DVector(action);
}

int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Search for the expansions
    search(map, planner);

    return 0;
}