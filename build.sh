#!/bin/sh
echo "Building RRT.cpp..."
g++ -Wall -c rrth.cpp -o rrt.o -std=c++14
echo "Building main.cpp..."
g++ -Wall -c main.cpp -o main.o -std=c++14
echo "Linking..."
g++ -Wall main.o rrt.o -o main -std=c++14
echo "Enter ./main to execute"
