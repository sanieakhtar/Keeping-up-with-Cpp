#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
using std::cout;
using std::ifstream;
using std::istringstream;
using std::sort;
using std::string;
using std::vector;

enum class State {kEmpty, kObstacle, kClosed, kPath, kStart, kFinish};

// directional deltas
const int delta[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

/**
 * The following four functions were written in lesson 2
 * They are general functions, no directly related to the A* algo implementation
 **/

/**
 * vector.push_back:
 * Adds a new element at the end of the vector, after its current last element.
 * The content of val is copied (or moved) to the new element.
 **/
vector<State> ParseLine(string line) {
    istringstream sline(line);
    int n;
    char c;
    vector<State> row;
    while (sline >> n >> c && c == ',') {
      if (n == 0) {
        row.push_back(State::kEmpty);
      } else {
        row.push_back(State::kObstacle);
      }
    }
    return row;
}

vector<vector<State>> ReadBoardFile(string path) {
  ifstream myfile (path);
  vector<vector<State>> board{};
  if (myfile) {
    string line;
    while (getline(myfile, line)) {
      vector<State> row = ParseLine(line);
      board.push_back(row);
    }
  }
  return board;
}

string CellString(State cell) {
  switch(cell) {
    case State::kObstacle: return "⛰️    ";
    case State::kPath: return "🚗   ";
    case State::kStart: return "🚦 ";
    case State::kFinish: return "🏁 ";
    default: return "0    "; 
  }
}

void PrintBoard(const vector<vector<State>> board) {
  for (int i = 0; i < board.size(); i++) {
    for (int j = 0; j < board[i].size(); j++) {
      cout << CellString(board[i][j]);
    }
    cout << "\n";
  }
}
// Until here.

/**
 * Heuristic cost function. Euclidean/Manhatten
 * distance from current node to goal node
 **/
int Heuristic (int currentX,int currentY,int goalX,int goalY) {
  return (abs(goalX-currentX) + abs(goalY-currentY));
}

/** 
 * This function adds nodes to the open vector (also initialized here)
 * Will also mark them as already visited -> kClosed
 * This is the expansion step of the algorithm. This is only carried out
 * if current node is not goal state. Therefore, current node is marked
 * as closed/visited and its neighbours are added to open list
**/
void AddToOpen(int x, int y, int g, int h, vector<vector<int>> &openNodes, vector<vector<State>> &board) {
  vector<int> currentNode = {x, y, g, h};
  openNodes.push_back(currentNode);
  board[x][y] = State::kClosed;
}

/**
 * Vector openNodes needs to be sorted according to f-value
 * Since the vector contains nodes of format {x,y,g,h}, no standard library can be used to sort them
 * This function compares the two nodes to determine their order and is a helper function for the
 * CellSort() function.
 **/
bool Compare(vector<int> nodeOne, vector<int> nodeTwo) {
  int fOne = nodeOne[2] + nodeOne[3]; // f1 = g1 + h1
  int fTwo = nodeTwo[2] + nodeTwo[3]; // f2 = g2 + h2
  return fOne > fTwo; 
  // bool to string to check?
}

/**
 * Sort the two-dimensional vector of ints in descending order.
 * 
 * sort(first, last, comp(optional)):
 * 
 * first, last: Define the range of the application of the sorting. Sorts the elements in the range [first,last) into ascending order.
 * The elements are compared using operator< for the first version, and comp for the second. They are random-access iterators referring to the
 * initial and final positions of the sequence to be sorted. The range used is [first,last), which contains all the elements between first and
 * last, including the first element but not the last
 * 
 * comp: Binary function that accepts two elements in the range as arguments, and returns a value convertible to bool.
 * The value returned indicates whether the element passed as first argument is considered to go before the second in the
 * specific strict weak ordering it defines. This can either be a function pointer or a function object.
 * 
 * If the value returned by the comp function is true, the first element is placed before the second. If false, then it is moved after the
 * second element. This is out the sorting is carried out here of the nodes, based on the f-value: f-values are passed to sort and compared using
 * the custom function. The entire node is passed to Compare(), the values are extracted, compared and true or false are returned.
 * 
 * Vector is sorted in DESCENDING order -> last element has the smalled heuristic 
 * 
 */
void CellSort(vector<vector<int>> *v) {
  sort(v->begin(), v->end(), Compare);
}

bool CheckValidCell (int cellX, int cellY, vector<vector<State>> &board) {
  // if (board[cellX][cellY] == State::kEmpty && cellX*cellY < board[0].size() * board.size()) {
  //   return true;
  // } else 
  //   return false;
  
  // OFFICIAL SOLUTION:
  bool on_grid_x = (cellX >= 0 && cellX < board.size());
  bool on_grid_y = (cellY >= 0 && cellY < board[0].size());
  if (on_grid_x && on_grid_y)
    return board[cellX][cellY] == State::kEmpty;
  return false;
}

void ExpandNeighbors(vector<int> &current, vector<vector<int>> &openNodes, vector<vector<State>> &grid, int goal[2]) {
  int x = current[0];
  int y = current[1];
  int g = current[2];

  // Loop through potential neighbours using your brain
  // Surprisingly, this was the trickiest part?! like wtf
  for (int i = 0; i < 4; i++) {
    int x2 = x + delta[i][0];
    int y2 = y + delta[i][1];

    // Check if neighbour cell is valid or not. If valid, calculate g and h values and add to open list.
    // If not, skip.
    if (CheckValidCell(x2, y2, grid)){
      int g2 = g + 1;
      int h2 = Heuristic(x2, y2, goal[0], goal[1]);

      AddToOpen(x2, y2, g2, h2, openNodes, grid);
    }
  }
}

/** 
 * Implementation of A* search algorithm
 */
vector<vector<State>> Search(vector<vector<State>> board, int start[2], int goal[2]) {
  // Create the vector of open nodes.
  vector<vector<int>> openNodes {};
  
  // Initialize the starting node
  int x = start[0];
  int y = start[1];
  int g = 0;
  int h = Heuristic(x, y, goal[0],goal[1]);
  AddToOpen(x, y, g, h, openNodes, board);

  // PERSONAL: write loop here to print out current open nodes

  // cout << openNodes.size() << "\n";

   while (openNodes.size() > 0) {
    CellSort(&openNodes);
    vector<int> currentNode = openNodes.back();
    // pick the last node since they are in descending order
    
    int currentX = currentNode[0];
    int currentY = currentNode[1];
    openNodes.pop_back();
    // need to remove open nodes to end search with or without solution. W/o this, search gets stuck in infinite loop.
    // lowest heuristic is removed since it will already be checked for goal state and added to path. Need to make space
    // for new lowest f-val node
    
    board[currentX][currentY] =  State::kPath;
    // this is set to path immediately since it has the lowest heuristic value and WILL be part of the path
    
    if (currentNode[0] == goal[0] && currentNode[1] == goal[1]) {
      board[start[0]][start[1]] = State::kStart;
      board[goal[0]][goal[1]] = State::kFinish;
      return board;
    }

    ExpandNeighbors(currentNode, openNodes, board, goal);
  }

  cout << "No path found!\n\n";
  return std::vector<vector<State>>{};
}


int main() {
  int init[2]{0,0};
  int goal[2]{4, 5};
  auto board = ReadBoardFile("inputFile.txt");
  auto solution = Search(board, init, goal);
  PrintBoard(board);
  cout << "\n\n";
  PrintBoard(solution);


  // shortcut to switch to terminal: alt+shift+t. still gotta figure out the return one
  // hitting the same shortcut works to bring back to editor but minimizes terminal
  // to compile, go to terminal and run magic 2.13-StoreGrid.cpp
  // to run, run ./a.out
  // ciaoooo
}