#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
#include <cmath>
#include <fstream>

using namespace std;

typedef struct Point{
	int x, y;

	bool operator<(Point other) const
	{
		return x < other.x && y < other.y;
	}
	bool operator>=(Point other) const
	{
		return x >= other.x && y >= other.y;
	}
	bool operator<=(Point other) const
	{
		return x <= other.x && y <= other.y;
	}
	bool operator==(Point other) const
	{
		return x == other.x && y == other.y;
	}
}Point;

class Node{
public:
	Point p;
	Node* parent;
	vector<Node*> child;
	double cost;

	Node(Point pt){
		p = pt;
		parent = NULL;
		cost = 0;
	}
	Node(){}
};

class RRT
{
public:
	RRT(int mn, int md, int mnd, int w, int h, vector<pair<Point, Point> > ob, Point s, Point d);
	double dist(Point a, Point b);
	Point getRandomPt();
	Point interpolate(Point a, Point b);
	bool inObstacle(Point pt);
	bool lineInObstacle(Point a, Point b);
	Node* findNearestNeighb(Point pt);
	vector<Node*> findNeighbours(vector<Node*> *neighbours, Point pt, Node* best);
	void rewire(vector<Node*> neighbours, Node node);
	void run();

	int max_nodes, max_dist, max_neigh_dist;
	int width, height;
	vector<pair<Point, Point> > obstacles;
	Point start, dest;
	vector<Node> rrtree;
};

#endif