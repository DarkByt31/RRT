#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include "rrt.h"
#include <stdexcept>
#include <string>
#include <array>
#include <memory>
#include <cstdio>


std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main()
{
	int i, n, choice;
	int max_nodes, max_dist, max_neigh_dist;
	int width, height;
	vector<pair<Point, Point> > obstacles;
	Point start, dest;

	cout<<"Choose from pre defined cases or press 0 to enter your own.\n";
	cout<<"Case 1:\n\
			space 100x100\n\
			max iter 500\n\
			max node dist 5\n\
			max neigh dist 8\n\
			start 0 0\n\
			dest 60 37\n\
			obstacle 20 40, 40 60; 60 20, 80 40\n";

	cout<<"Case 2:\n\
			space 200x200\n\
			max iter 1000\n\
			max node dist 6\n\
			max neigh dist 10\n\
			start 50 50\n\
			dest 180 180\n\
			obstacle 150 150, 170 170\n";
			
	cout<<"Case 3:\n\
			space 640x480\n\
			max iter 5000\n\
			max node dist 7\n\
			max neigh dist 15\n\
			start 0 0\n\
			dest 630 470\n\
			obstacle 100 150, 200 200; 250 300, 450 400; 500 100, 600 250\n";

	cin>>choice;

	if(choice == 1){
		width = 100, height = 100, max_nodes = 500, max_dist = 5, max_neigh_dist = 8;
		start.x = 0, start.y = 0, dest.x = 60, dest.y = 37;
		obstacles = { make_pair(Point{20, 40}, Point{40, 60}), 
						make_pair(Point{60, 20}, Point{80, 40}) };
	}
	else if(choice == 2){
		width = 200, height = 200, max_nodes = 1000, max_dist = 6, max_neigh_dist = 10;
		start.x = 50, start.y = 50, dest.x = 180, dest.y = 180;
		obstacles = { make_pair(Point{150, 150}, Point{170, 170}) };
	}
	else if(choice == 3){
		width = 640, height = 480, max_nodes = 5000, max_dist = 7, max_neigh_dist = 15;
		start.x = 0, start.y = 0, dest.x = 630, dest.y = 470;
		obstacles = { make_pair(Point{100, 150}, Point{200, 200}), 
						make_pair(Point{250, 300}, Point{450, 400}),
						make_pair(Point{500, 100}, Point{600, 250}) };
	
	}
	else{
		cout<<"Enter following parameters: \n";
		cout<<"Enter width and height of space: ";
		cin>>width>>height;
		cout<<"Starting point(x y): ";
		cin>>start.x>>start.y;
		cout<<"Ending Point(x y): ";
		cin>>dest.x>>dest.y;
		cout<<"Max number of iterations: ";
		cin>>max_nodes;
		cout<<"Max distance between each node: ";
		cin>>max_dist;
		cout<<"Max distance between neighbouring node: ";
		cin>>max_neigh_dist;

		cout<<"Number of obstacles: ";
		cin>>n;
		for(i=0; i<n; i++){
			cout<<"(lower point, higher point) x1 y1 x2 y2: ";
			int x1, y1, x2, y2;
			cin>>x1>>y1>>x2>>y2;
			obstacles.push_back(make_pair(Point{x1, y1}, Point{x2, y2}));
		}
	}

	RRT rrt(max_nodes, max_dist, max_neigh_dist, width, height, obstacles, start, dest);
	rrt.run();

	// Plot points
	string command = "gnuplot -c gp " + to_string(width) + " " + to_string(height);
	exec(command.c_str());

	return 0;
}

/* 
* write instructions on how to build and the requirements
*/