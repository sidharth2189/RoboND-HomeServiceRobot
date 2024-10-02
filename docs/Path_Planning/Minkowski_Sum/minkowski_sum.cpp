#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

// Print 2D vectors coordinate values
void print2DVector(vector<vector<float> > vec)
{
     // Sorting the vector for grading purpose
    sort(vec.begin(), vec.end());
    for (int i = 0; i < vec.size(); ++i) {
        for (int j = 0; j < vec[0].size(); ++j) {
                cout << vec[i][j] << "  ";
        }
        cout << endl;
    }
}

// ***TODO: Check for duplicate coordinates inside a 2D vector and delete them*** //
vector<vector<float> > delete_duplicate(vector<vector<float> > C)
{   
    std::sort(C.begin(), C.end());
    C.erase(std::unique(C.begin(), C.end()), C.end());
    return C;
}

// ***TODO: Compute the Minkowski Sum of two vectors***//
vector<vector<float> > minkowski_sum(vector<vector<float> > A, vector<vector<float> > B)
{
    vector<vector<float> > C;
    for (auto valA: A)
        for (auto valB: B)
            C.push_back({valA[0] + valB[0], valA[1] + valB[1]});

    C = delete_duplicate(C);
    return C;
}

int main()
{
    // ***TODO: Define the coordinates of triangle A and B using 2D vectors*** //
    vector<vector<float> > A{{1.25, 0.0}, {0.25, 1.0}, {0.25, -1.0}};
    vector<vector<float> > B{{0.0, 0.0}, {1.0, 1.0}, {1, -1.0}};
    
    // Compute the minkowski sum of triangle A and B
    vector<vector<float> > C;
    C = minkowski_sum(A, B);

    // Print the resulting vector
    print2DVector(C);

    return 0;
}