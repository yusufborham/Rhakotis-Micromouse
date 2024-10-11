#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <cstring>
#include <list>
#include <iostream>
#include "API.h"
#include <algorithm>
#include <map>
#include <string>

using namespace std;

#define MAZE_WIDTH 18
#define MAZE_HEIGHT 18
#define INF 999999

//int visited[324] = {0};  // keep track of visited cells
const char directions[4] = {'n', 'e', 's', 'w'};  // list of direction labels
int dir[4] = {18, 1, -18, -1};  // list of directional offset for navigation
//std::deque<int> queue1; // Placeholder for deque
//extern std::deque<int> queue;
//int orient = 0;

bool maze[MAZE_WIDTH * MAZE_HEIGHT][4]; // 2D array to store the presence of walls in each cell
int goal_cells []= {152, 153, 170, 171}; // my goals in the maze
char MyOrient = 'n'; // initial orient direction
char DesiredOrient = 'n'; // initial desired direction


int ManhDis[MAZE_HEIGHT][MAZE_WIDTH] = {
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8},
    {9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    {11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12},
    {13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16}
};
 
//global variables for GetNeighbours
int northcell;
int eastcell;
int southcell;
int westcell;

// void log(const std::string& message) {
//     std::cerr << message << std::endl;
//     std::cerr.flush();
// }
#define DEQUE_SIZE 324 // Define deque size

class Deque {
  private:
    int data[DEQUE_SIZE];
    int head, tail, count;

  public:
    Deque() {
      head = 0;
      tail = -1;
      count = 0;
    }

    bool isFull() {
      return count == DEQUE_SIZE;
    }

    bool isEmpty() {
      return count == 0;
    }

    void pushFront(int value) {
      if (isFull()) {
        return;
      }
      head = (head - 1 + DEQUE_SIZE) % DEQUE_SIZE;
      data[head] = value;
      count++;
    }

    void pushBack(int value) {
      if (isFull()) {
        return;
      }
      tail = (tail + 1) % DEQUE_SIZE;
      data[tail] = value;
      count++;
    }

    int popFront() {
      if (isEmpty()) {
        return -1;
      }
      int value = data[head];
      head = (head + 1) % DEQUE_SIZE;
      count--;
      return value;
    }

    int popBack() {
      if (isEmpty()) {
        return -1;
      }
      int value = data[tail];
      tail = (tail - 1 + DEQUE_SIZE) % DEQUE_SIZE;
      count--;
      return value;
    }

    int front() {
      if (isEmpty()) {
        return -1;
      }
      return data[head];
    }

    int back() {
      if (isEmpty()) {
        return -1;
      }
      return data[tail];
    }
};

#define MAX_SET_SIZE 324  // Maximum number of elements in the set

typedef struct {
    int elements[MAX_SET_SIZE];  // Array to store elements
    int size;                    // Current size of the set
} Set;

// Function to initialize the set
void initSet(Set* s) {
    s->size = 0;
}

//Function to check if an element is present in the set
bool isMember(Set* s, int element) {
    for (int i = 0; i < s->size; i++) {
        if (s->elements[i] == element) {
            return true;
        }
    }
    return false;
}

// Function to add an element to the set (if it doesn't exist already)
void addElement(Set* s, int element) {
    if (!isMember(s, element)) {
        if (s->size < MAX_SET_SIZE) {
            s->elements[s->size++] = element;
        } else {
          //  printf("Set is full, cannot add more elements.\n");
        }
    } else {
       // printf("Element %d is already in the set.\n", element);
    }
}

// Function to remove an element from the set
// void removeElement(Set* s, int element) {
//     for (int i = 0; i < s->size; i++) {
//         if (s->elements[i] == element) {
//             // Shift elements to the left to fill the gap
//             for (int j = i; j < s->size - 1; j++) {
//                 s->elements[j] = s->elements[j + 1];
//             }
//             s->size--;
//             return;
//         }
//     }
//    // printf("Element %d not found in the set.\n", element);
// }

// Function to display the set
// void displaySet(Set* s) {
//    // printf("Set elements: { ");
//     for (int i = 0; i < s->size; i++) {
//        // printf("%d ", s->elements[i]);
//     }
//   //  printf("}\n");
// }

Deque queue1;



void log(const std::string &text)
{
    std::cerr << text << std::endl;
}
void logNum(const int &text)
{
    std::cerr << text << std::endl;
}

void genDefManhDis() {
    for (int i = 0; i < 18; i++) {
        for (int j = 0; j < 18; j++) {
            ManhDis[i][j] = INF;
        }
    }
}

void displayManhDis() {
    for (int i = 1; i < 17; i++) {
        for (int j = 1; j < 17; j++) {
           if (ManhDis[i][j] == INF){
            continue;
           }
           API::setText(j - 1, i - 1, ManhDis[i][j]); // Placeholder for API function
            // printf("ManhDis[%d][%d] = %d\n", i - 1, j - 1, ManhDis[i][j]); // For demonstration
        }
    }
}



void initializeMaze() {
    for (int i = 0; i < 18; i++) {
        for (int j = 0; j < 4; j++) {
            maze[i][j] = true;
            maze[i * 18][j] = true;
            maze[i + 306][j] = true;
            maze[i * 18 + 17][j] = true;
        }
        maze[i + 18][2] = true;
        maze[i * 18 + 1][3] = true;
        maze[i * 18 + 16][1] = true;
        maze[i + 288][0] = true;
    }
}


// void Turn(char MyOrient, char DesiredOrient) {
void Turn(){
    int Orientations[4] = { 0, 1, 2, 3 };
    // int MyOrientIndex = std::distance(directions, std::find(std::begin(directions), std::end(directions), MyOrient));
    int MyOrientIndex = -1;
    for (int i = 0; i < sizeof(directions) / sizeof(directions[0]); i++) {
        if (directions[i] == MyOrient) {
            MyOrientIndex = i;  // Found the index
            break;  // Exit the loop once found
        }
    }
    // log("In Turn");
    // log("-------------------------------------");
    // log(std::to_string(MyOrientIndex));
    // int DesiredOrientIndex = std::distance(directions, std::find(std::begin(directions), std::end(directions), DesiredOrient));
    int DesiredOrientIndex = -1;
    for (int i = 0; i < sizeof(directions) / sizeof(directions[0]); i++) {
        if (directions[i] == DesiredOrient) {
            DesiredOrientIndex = i;  // Found the index
            break;  // Exit the loop once found
        }
    }
    // log(std::to_string(DesiredOrientIndex));
    // Get the current and target orientations
    int current = Orientations[MyOrientIndex];
    int target = Orientations[DesiredOrientIndex];

    // Calculate the difference in orientations
    int diff = (target - current + 4) % 4;  // Adding 4 to ensure a non-negative result before modulo
    // log(std::to_string(diff));
    // Apply the minimal number of turns
    if (diff == 1) {
        API::turnRight();
    } else if (diff == 3) {  // Equivalent to -1 mod 4
        API::turnLeft();
    } else if (diff == 2) {
        API::turnRight();
        API::turnRight();
    }

    // Update the orientation after turning
    MyOrient = DesiredOrient;
    // log("Myorient in turn");
    // log(std::to_string(MyOrient));
    // Move forward after turning
    API::moveForward();
}

void checkAccessibilty(int index) {
    
    // log("Myorient in checkAccessibilty");
    // log(std::to_string(MyOrient));

    // int orient_index ; //???????
    // int orient_index = std::find(directions.begin(), directions.end(), orient);
    // find bt-return pointer lel mkan el orient ely l2ah 
    // distance between directions (pointer to first of array)  & pointer ely hyrga3 mn find == index
    // log("In checkAccessibility");
    // int orient_index = std::distance(directions, std::find(std::begin(directions), std::end(directions), MyOrient));
    int orient_index = -1;
    for (int i = 0; i < sizeof(directions) / sizeof(directions[0]); i++) {
        if (directions[i] == MyOrient) {
            orient_index = i;  // Found the index
            break;  // Exit the loop once found
        }
    }
    // log(std::to_string(orient_index));
    int row = index / MAZE_WIDTH;
    int col = index % MAZE_WIDTH;
    
    // Handle wall front
    if (API::wallFront()) {
       API::setWall(col - 1, row - 1, directions[orient_index]);
        maze[index][orient_index] = true;
        maze[index + dir[orient_index]][(orient_index + 2) % 4] = true;
    }

    // Handle wall left
    if (API::wallLeft()) {
       API::setWall(col - 1, row - 1, directions[(orient_index - 1 + 4) % 4]);
        maze[index][(orient_index - 1 + 4) % 4] = true;
        maze[index + dir[(orient_index - 1 + 4) % 4]][(orient_index + 1) % 4] = true;
    }

    // Handle wall right
    if (API::wallRight()) {
       API::setWall(col - 1, row - 1, directions[(orient_index + 1) % 4]);
        maze[index][(orient_index + 1) % 4] = true;
        maze[index + dir[(orient_index + 1) % 4]][(orient_index + 3) % 4] = true;
    }
}



void GetNeighbours(int index) {
    int row = (index / MAZE_WIDTH);
    int col = index % MAZE_HEIGHT;

    // neighbours[0] = ManhDis[row + 1][col]; //northcell
    // neighbours[1] = ManhDis[row][col + 1]; //eastcell
    // neighbours[2] = ManhDis[row - 1][col]; //southcell
    // neighbours[3] = ManhDis[row][col - 1]; //westcell

      northcell = ManhDis[row+1][col];
      eastcell = ManhDis[row][col+1];
      southcell = ManhDis[row-1][col];
      westcell = ManhDis[row][col-1];
    //   log("In GetNeighbours: ");
    //   log(std::to_string(northcell)); 
    //   log(std::to_string(eastcell));
    //   log(std::to_string(southcell));
    //   log(std::to_string(westcell));

}
bool visited[324];

void updateMaze2() {

    // log("Myorient in updateMaze2 ");
    // log(std::to_string(MyOrient));
    // std::deque<int> queue; 
 
    for (int i = 0; i < 324; i++) {
        visited[i] = false;
    }

    genDefManhDis();
    displayManhDis();
    int cnt = 0;
    for (int i = 0; i <sizeof(goal_cells)/sizeof(goal_cells[0]); i++) {
        queue1.pushBack(goal_cells[i]);
        cnt++ ;
        ManhDis[(goal_cells[i] / MAZE_WIDTH)][goal_cells[i] % MAZE_WIDTH] = 0;
    }
    // log("SIZE OF QUEUE");
    // logNum(cnt);

    while (! queue1.isEmpty()) {
        // log("queue:");
        // for(int j =0 ; j<cnt; j++){
        //         logNum(queue1[j]);
        // }
        // log("SIZE OF QUEUE");
        // logNum(sizeof(queue1)/sizeof(int));
        int frontcell = queue1.front();
        // log("-----------------------------------------\n frontcell");
        // logNum(frontcell);
        queue1.popFront();
        cnt--;
        if (visited[frontcell]) {
           API::setColor((frontcell % MAZE_WIDTH) - 1, (frontcell / MAZE_WIDTH) - 1, 'g');
            continue;
        }
        API::setColor((frontcell % MAZE_WIDTH) - 1, (frontcell / MAZE_WIDTH) - 1, 'o');
        visited[frontcell] = true;
        int row = frontcell / MAZE_WIDTH;
        int col = frontcell % MAZE_WIDTH;
        //set<int> neighbours ;
        Set neighbours;

        //int north, east, south, west;
        GetNeighbours(frontcell);
        int N_wall = maze[frontcell][0];
        int E_wall = maze[frontcell][1];
        int S_wall = maze[frontcell][2];
        int W_wall = maze[frontcell][3];
        // log(to_string(queue1.size()));
        if (!N_wall) {
           // neighbours.insert(northcell);
           addElement(&neighbours,northcell);
            queue1.pushBack(frontcell + 18);
            cnt++ ;
            API::setColor(((frontcell + 18) % MAZE_WIDTH) - 1, ((frontcell + 18) / MAZE_WIDTH) - 1, 'y');
            ManhDis[row + 1][col] = min(ManhDis[row][col] + 1, ManhDis[row + 1][col]);
        }
        // log("done 1");
        if (!E_wall) {
           // neighbours.insert(eastcell);
           addElement(&neighbours,eastcell);
            queue1.pushBack(frontcell + 1);
            cnt++;
            API::setColor(((frontcell + 1) % MAZE_WIDTH) - 1, ((frontcell + 1) / MAZE_WIDTH) - 1, 'y');
            ManhDis[row][col + 1] = min(ManhDis[row][col] + 1, ManhDis[row][col + 1]);
        }
        // log("done 2");
        if (!S_wall) {
            //neighbours.insert(southcell);
            addElement(&neighbours,southcell);
            queue1.pushBack(frontcell - 18);
            cnt++;
           API::setColor(((frontcell - 18) % MAZE_WIDTH) - 1, ((frontcell - 18) / MAZE_WIDTH) - 1, 'y');
            ManhDis[row - 1][col] = min(ManhDis[row][col] + 1, ManhDis[row - 1][col]);
        }
        // log("done 3");
        if (!W_wall) {
            // neighbours.insert(westcell);
            addElement(&neighbours,westcell);
            queue1.pushBack(frontcell - 1);
            cnt++;
           API::setColor(((frontcell - 1) % MAZE_WIDTH) - 1, ((frontcell - 1) / MAZE_WIDTH) - 1, 'y');
            ManhDis[row][col - 1] = min(ManhDis[row][col] + 1, ManhDis[row][col - 1]);
        }
        // log("done 4");
        displayManhDis();
        API::setColor((frontcell % MAZE_WIDTH) - 1, (frontcell / MAZE_WIDTH) - 1, 'g');
    }
    for (int i = 1; i <= 17; i++) {
        for (int j = 1; j <= 17; j++) {
            API::setColor(j - 1, i - 1, 'k');
        }
    }
}

void move(int index) {
    
    // log("Myorient in move: ");
    // log(std::to_string(MyOrient));
    int counter = 0;
    // log("In'move:");
    while (index != 152 || index != 153 || index != 170 || index != 171) {
        // log("Inside while in move:");
        int row = (index / MAZE_HEIGHT);
        int col = index % MAZE_WIDTH;
       // int north, east, south, west;
        GetNeighbours(index);
        // north = northcell;
        // east = eastcell;
        // south = southcell;
        // west = westcell;
        // log("logging neighbours again in move:");
        // log(std::to_string(northcell)); 
        // log(std::to_string(eastcell));
        // log(std::to_string(southcell));
        // log(std::to_string(westcell));
        checkAccessibilty(index);

        if ((!maze[index][0]) && (northcell == ManhDis[row][col] - 1)) {
            // MyOrient = rover_orient ;
            // log("Myorient in move : north before: ");
            // log(std::to_string(MyOrient));
            DesiredOrient = 'n';
            // log("turn north");
            Turn();
            // log("Myorient in move : north after: ");
            // log(std::to_string(MyOrient));
            
            // log("turn north done");
            // rover_orient = 'n';
            // MyOrient = 'n';
            index += 18;
        } else if ((!maze[index][1]) && (eastcell == ManhDis[row][col] - 1)) {
            // Turn(&rover_orient, "e");
            // MyOrient = rover_orient ;
            // log("Myorient in move : east before: ");
            // log(std::to_string(MyOrient));
            DesiredOrient = 'e';
            // log("turn east");
            Turn();
            // log("Myorient in move : east after: ");
            // log(std::to_string(MyOrient));
            // log("turn east done");
            // rover_orient = 'e';
            // MyOrient = 'e';
            index += 1;
        } else if ((!maze[index][3]) && (westcell == ManhDis[row][col] - 1)) {
            // Turn(&rover_orient, "w");
            // MyOrient = rover_orient ;
            // log("Myorient in move : west before: ");
            // log(std::to_string(MyOrient));
            DesiredOrient = 'w';
            // log("turn west");
            Turn();
            // log("Myorient in move : west after: ");
            // log(std::to_string(MyOrient));
            // log("turn west done");
            // rover_orient = 'w';
            // MyOrient = 'w';
            index -= 1;
        } else if ((!maze[index][2]) && (southcell == ManhDis[row][col] - 1)) {
            // Turn(&rover_orient, "s");
            // MyOrient = rover_orient ;
            // log("Myorient in move : south before: ");
            // log(std::to_string(MyOrient));
            DesiredOrient = 's';
           // log("turn south");
            Turn();
            // log("Myorient in move : south after: ");
            // log(std::to_string(MyOrient));
            // log("turn south done");
            // rover_orient = 's';
            // MyOrient = 's';
            index -= 18;
        } else {
            //log(ManhDis[row][col]);
            updateMaze2();
            counter += 1;
        }
    }
}

int main()
{
    initializeMaze();
    int index = 19;
    // char rover_orient = 'n';
    // MyOrient = 'n'
    //log("starting to move");
    move(index);

}