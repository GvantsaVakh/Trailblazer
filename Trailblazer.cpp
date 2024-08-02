/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include "random.h"

using namespace std;

//node for shortestPath
struct Node {
    Node* parent;
    double dist;
    int color;
    Loc loc;
};

bool changeLoc(Loc curr,Loc& l, Grid<double>& world, int i, Vector<int> changeLocRow, Vector<int> changeLocCol) {
    int row = curr.row + changeLocRow[i];
    int col = curr.col + changeLocCol[i];
    if (!world.inBounds(row, col)) {
        return false;
    }
    l.row = row;
    l.col = col;
    return true;;
}

void fillGrid(Grid<Node*>& g) {
    for (int r = 0; r < g.numRows(); r++) {
        for (int c = 0; c < g.numCols(); c++) {
            Node* newNode = new Node;
            newNode->color = 0;
            Loc l;
            l.row = r;
            l.col = c;
            newNode->loc = l;
            g[r][c] = newNode;
        }
    }
}

void fillChanger(Vector<int>& changeLocRow, Vector<int>& changeLocCol) {
    changeLocCol += -1, 0, 1, 1, 1, 0, -1, -1;
    changeLocRow += -1, -1, -1, 0, 1, 1, 1, 0;
}

void fillRes(Vector<Loc>& res, Node* endNode) {
    if (endNode == NULL) {
        return;
    }
    res.insert(0, endNode->loc);
    fillRes(res, endNode->parent);
}

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start,
             Loc end,
             Grid<double>& world,
             double costFn(Loc from, Loc to, Grid<double>& world), double heuristic(Loc start, Loc end, Grid<double>& world)) {
    Vector<int> changeLocRow;
    Vector<int> changeLocCol;

    Grid<Node*> myGrid(world.numRows(), world.numCols());

    fillGrid(myGrid);

    fillChanger(changeLocRow, changeLocCol);
    
    TrailblazerPQueue<Node*> queue;
    Node* startNode = myGrid[start.row][start.col];
    startNode->loc = start;
    startNode->parent = NULL;
    startNode->color = 1;
    startNode->dist = 0;
    myGrid[start.row][start.col] = startNode;
    colorCell(world, start, YELLOW);
    queue.enqueue(startNode, heuristic(start, end, world));
    Node* endNode = NULL;
    while (!queue.isEmpty()) { 
        Node* currNode = queue.dequeueMin();  
        Loc currLoc = currNode->loc;    
        colorCell(world, currLoc, GREEN);   
        currNode->color = 2;
        double dist = currNode->dist;
        if (currLoc == end) {
            endNode = currNode;
            break;
        }
        for (int i = 0; i < 8; i++) {           
            Loc newLoc;
            if (changeLoc(currLoc, newLoc, world, i, changeLocRow, changeLocCol)) {
                Node* neighbour = myGrid[newLoc.row][newLoc.col];
                double crossCost = costFn(currLoc, newLoc, world);
                if (neighbour->color == 0) {
                    neighbour->color = 1;
                    neighbour->dist = dist + crossCost;
                    neighbour->parent = currNode;
                    colorCell(world, newLoc, YELLOW);                
                    queue.enqueue(neighbour, neighbour->dist + heuristic(newLoc, end, world));
                }
                else if (neighbour->color == 1 && neighbour->dist > (dist + crossCost)) {
                    neighbour->dist = dist + crossCost;
                    neighbour->parent = currNode;
                    queue.decreaseKey(neighbour, neighbour->dist + heuristic(newLoc, end, world));
                }
            }
        }
    }
    Vector<Loc> res;
    fillRes(res, endNode);
    return res;
}


void fillMapAndQueue(TrailblazerPQueue<Edge>& queue, Map<Loc, Loc>& neighboursMap, int numRows, int numCols) {
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            Loc l1;
            l1.row = i;
            l1.col = j;

            neighboursMap[l1] = l1;

            Loc l2;
            l2.row = i;
            l2.col = j + 1;

            Loc l3;
            l3.row = i + 1;
            l3.col = j;

            if (l2.row < numRows && l2.col < numCols) {
                Edge e;
                e.start = l1;
                e.end = l2;
                double weight = randomReal(0, 1);
                queue.enqueue(e, weight);
            }
            if (l3.row < numRows && l3.col < numCols) {
                Edge e;
                e.start = l1;
                e.end = l3;
                double weight = randomReal(0, 1);
                queue.enqueue(e, weight);
            }
        }
    }
}

Loc find(Loc a, Map<Loc, Loc>& parents) {
    if (parents[a] == a) {
        return a;
    }
    return find(parents[a], parents);
}

bool sameCluster(Loc start, Loc end, Map<Loc, Loc> parents) {
    return find(start, parents) == find(end, parents);
}

void unite(Loc start, Loc end, Map<Loc, Loc>& parents) {
    Loc rootStart = find(start, parents);
    Loc rootEnd = find(end, parents);

    if (rootStart != rootEnd) {
        parents[rootEnd] = rootStart;
    }

}

Set<Edge> createMaze(int numRows, int numCols) {
    Set<Edge> edges;
    Set<Loc> locs;
    Map <Loc, Loc> parents;
    TrailblazerPQueue<Edge> queue;
    fillMapAndQueue(queue, parents, numRows, numCols);
    int numClusters = numRows * numCols;
    while (numClusters > 1) {
        Edge curr = queue.dequeueMin();
        Loc startLoc = curr.start;
        Loc endLoc = curr.end;

        if (!sameCluster(startLoc, endLoc, parents)) {
              unite(startLoc, endLoc, parents);
            edges.add(curr);
            numClusters--;
        }
    }
    return edges;
}
