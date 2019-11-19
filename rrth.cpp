#include "rrt.h"

RRT::RRT(int mn, int md, int mnd, int w, int h, vector<pair<Point, Point> > ob, Point s, Point d){
	max_nodes = mn;
	max_dist = md;
	max_neigh_dist = mnd;
	width = w;
	height = h;
	obstacles = ob;
	start = s;
	dest = d;

	rrtree.reserve(max_nodes+2);
}

// Returns euclidean distance between two Points
double RRT::dist(Point a, Point b){
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

// Returns a valid random point
Point RRT::getRandomPt(){
	// generate random Point
	Point rand_pt = Point{drand48() * width, drand48() * height};
	while(rand_pt >= Point{width, height} && rand_pt <= Point{0, 0})
		rand_pt = Point{drand48() * width, drand48() * height};

	return rand_pt;
}

// Returns point within the radius
Point RRT::interpolate(Point a, Point b){
	if(dist(a, b) < max_dist)	return a;

	double angle = atan2(a.y - b.y, a.x - b.x);
	return Point{b.x + max_dist*cos(angle), b.y + max_dist*sin(angle)};
}

// Return true if the point is inside obstacle
bool RRT::inObstacle(Point pt){
	for(int i=0; i<obstacles.size(); i++)
		if(pt >= obstacles[i].first && pt <= obstacles[i].second){
			return true;
		}
	return false;
}

// Returns true if two lines intersect
bool lineIntersect(int x00, int y00, int x10, int y10, int x01, int y01, int x11, int y11){
	double d = x11*y01 - x01*y11;
	double s = (1/d) * ((x00 - x10)*y01 - (y00 - y10)*x01);
	double t = -(1/d) * (-(x00 - x10)*y11 + (y00 - y10)*x11);
	if((s >= 0 && s <= 1) && (t >= 0 && t <= 1))	return true;
	return false;
}

// Returns true if the line segment passes through obstacle
bool RRT::lineInObstacle(Point a, Point b){
	int x00 = a.x, y00 = a.y, x10 = b.x-x00, y10 = b.y-y00;
	for(int i=0; i<obstacles.size(); i++){
		// lower side parallel to x axis
		int x01 = obstacles[i].first.x, y01 = obstacles[i].first.y;
		int x11 = obstacles[i].second.x - x01;
		int y11 = obstacles[i].first.y - y01;
		bool intersect = lineIntersect(x00, y00, x10, y10, x01, y01, x11, y11);
		if(intersect)	return true;

		// right side parallel to y axis
		x01 = obstacles[i].second.x, y01 = obstacles[i].first.y;
		x11 = obstacles[i].second.x - x01, y11 = obstacles[i].second.y - y01;
		intersect = lineIntersect(x00, y00, x10, y10, x01, y01, x11, y11);
		if(intersect)	return true;

		// upper side parallel to x axis
		x01 = obstacles[i].first.x, y01 = obstacles[i].second.y;
		x11 = obstacles[i].second.x - x01, y11 = obstacles[i].second.y - y01;
		intersect = lineIntersect(x00, y00, x10, y10, x01, y01, x11, y11);
		if(intersect)	return true;

		// left side parallel to y axis
		x01 = obstacles[i].first.x, y01 = obstacles[i].first.y;
		x11 = obstacles[i].first.x - x01, y11 = obstacles[i].second.y - y01;
		intersect = lineIntersect(x00, y00, x10, y10, x01, y01, x11, y11);
		if(intersect)	return true;
	}
	return false;
}

// Return nearest neighbour of a Point
Node* RRT::findNearestNeighb(Point pt){
	Node* nn = &rrtree[0];
	for(int i=1; i<rrtree.size(); i++){
		if(dist(pt, nn->p) > dist(pt, rrtree[i].p)){
			nn = &rrtree[i];
		}
	}

	return nn;
}

// Return best node, with least cost, and a list of neighbour nodes
vector<Node*> RRT::findNeighbours(vector<Node*> *neighbours, Point pt, Node* best){
	vector<Node*> n2;
	for(int i=0; i<rrtree.size(); i++){
		if(dist(pt, rrtree[i].p) < max_neigh_dist){
			neighbours->push_back(&rrtree[i]);	n2.push_back(&rrtree[i]);
			// check for the best node
			if(best->cost + dist(best->p, pt) > rrtree[i].cost + dist(rrtree[i].p, pt))
				*best = rrtree[i];
		}
	}
	return n2;
}

// Rewire the tree
void RRT::rewire(vector<Node*> neighbours, Node node){
	for(int i=0; i<neighbours.size(); i++){
		if(node.cost + dist(node.p, neighbours[i]->p) < neighbours[i]->cost){
			neighbours[i]->cost = node.cost + dist(node.p, neighbours[i]->p);
			Node* temp = neighbours[i]->parent;
			temp->parent = &node;
		}
	}
}

void RRT::run()
{
	int i = max_nodes;

	rrtree.push_back(Node(start));
	rrtree[0].parent = &rrtree[0];
	while(i){
		cout<<"ITERATION: "<<max_nodes - i;
		Point rand_pt = getRandomPt();

		// find nearest neighbour to the random point
		Node* nearestNeighb = findNearestNeighb(rand_pt);
		rand_pt = interpolate(rand_pt, nearestNeighb->p);
		if(inObstacle(rand_pt))	continue;

		cout<<" finding neighbours... ";
		vector<Node*> neighbours;
		Node* best = nearestNeighb;
		// find best node and neighbours		
		for(int i=0; i<rrtree.size(); i++){
			if(dist(rand_pt, rrtree[i].p) < max_neigh_dist){
				neighbours.push_back(&rrtree[i]);
				// check for the best node
				if(best->cost + dist(best->p, rand_pt) > rrtree[i].cost + dist(rrtree[i].p, rand_pt))
					best = &rrtree[i];
			}
		}

		// Check if line passes thorugh the obstacle
		if(lineInObstacle(rand_pt, best->p))	continue;

		/*for(int i=0; i<neighbours.size(); i++)
			cout<<neighbours[i]->p.x<<", "<<neighbours[i]->p.y<<endl;*/

		Node newnode(rand_pt);
		newnode.parent = best;
		newnode.cost = best->cost + dist(rand_pt, best->p);
		rrtree.push_back(newnode);
		i--;

		cout<<" rewiring... ";
		// rewire the tree
		rewire(neighbours, newnode);

		//cout<<"here"<<endl;
		if(dist(rand_pt, dest) <= max_dist){
			cout<<"\n*******\n"
				<<"Reached in iterations: "<<max_nodes - i<<"\n"
				<<"********\n";
			break;
		}
	}

	Node* n = findNearestNeighb(dest);
	cout<<"\nLeast distance between destination: "<<dist(dest, n->p)<<endl;


	ofstream outfile;
	outfile.open("points.txt", ios::out | ios::trunc );
	for(int i=0; i<rrtree.size(); i++)
		outfile<<rrtree[i].p.x<<", "<<rrtree[i].p.y<<endl;

	outfile.close();
}
