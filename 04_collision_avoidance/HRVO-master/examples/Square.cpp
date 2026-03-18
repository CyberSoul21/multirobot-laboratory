/*
 * Square.cpp
 * HRVO Library
 *
 * Modified example:
 * - Agents start on the border of a square
 * - Goal positions are distributed on the contour of the letter W
 */

#ifndef HRVO_OUTPUT_TIME_AND_POSITIONS
#define HRVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>

#if HRVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <HRVO.h>
#include <fstream>

using namespace hrvo;

const float HRVO_TWO_PI = 6.283185307179586f;

//creates a point between two points.
Vector2 interpolate(const Vector2 &p1, const Vector2 &p2, float t)
{
    return p1 + t * (p2 - p1);
}

std::vector<Vector2> makeSquareBorder(std::size_t numAgents, float L)
{
    //center at (0,0)
    //half side length = L = 200
    //divide the robots among the 4 edges.
    std::vector<Vector2> pts;
    pts.reserve(numAgents);

    for (std::size_t i = 0; i < numAgents; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(numAgents);
        float p = t * 4.0f;
        Vector2 pos;

        if (p < 1.0f) {
            // Bottom side: left -> right
            pos = Vector2(-L + 2.0f * L * p, -L);
        }
        else if (p < 2.0f) {
            // Right side: bottom -> top
            pos = Vector2(L, -L + 2.0f * L * (p - 1.0f));
        }
        else if (p < 3.0f) {
            // Top side: right -> left
            pos = Vector2(L - 2.0f * L * (p - 2.0f), L);
        }
        else {
            // Left side: top -> bottom
            pos = Vector2(-L, L - 2.0f * L * (p - 3.0f));
        }

        pts.push_back(pos);
    }

    return pts;
}

std::vector<Vector2> makeLetterW(std::size_t numAgents)
{
    std::vector<Vector2> pts;
    pts.reserve(numAgents);

    // Key points of the capital letter W
    Vector2 A(-150.0f,  150.0f);   // top-left
    Vector2 B( -75.0f, -150.0f);   // bottom-left
    Vector2 C(   0.0f,   20.0f);   // middle peak
    Vector2 D(  75.0f, -150.0f);   // bottom-right
    Vector2 E( 150.0f,  150.0f);   // top-right

    for (std::size_t i = 0; i < numAgents; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(numAgents);
        float p = t * 4.0f;

        if (p < 1.0f) {
            pts.push_back(interpolate(A, B, p));
        }
        else if (p < 2.0f) {
            pts.push_back(interpolate(B, C, p - 1.0f));
        }
        else if (p < 3.0f) {
            pts.push_back(interpolate(C, D, p - 2.0f));
        }
        else {
            pts.push_back(interpolate(D, E, p - 3.0f));
        }
    }

    return pts;
}

int main()
{
	Simulator simulator;
	
	std::ofstream outfile;
	outfile.open("resultSquare.txt"); // open a file in write mode.
	std::size_t numAgents = 250;

	simulator.setTimeStep(0.25f);
	simulator.setAgentDefaults(15.0f, 10, 1.5f, 1.5f, 1.0f, 2.0f);

    std::vector<Vector2> startPositions = makeSquareBorder(numAgents, 200.0f);
    std::vector<Vector2> goalPositions  = makeLetterW(numAgents);    

	for (std::size_t i = 0; i < numAgents; ++i) {
		simulator.addAgent(startPositions[i], simulator.addGoal(goalPositions[i]));
	}

	do {

#if HRVO_OUTPUT_TIME_AND_POSITIONS
		std::cout << simulator.getGlobalTime();
		// write time to file
        outfile << simulator.getGlobalTime();		

		for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
			// write each agent position to file
            outfile << " " << simulator.getAgentPosition(i);
			std::cout << " " << simulator.getAgentPosition(i);
		}
		outfile << std::endl;
		std::cout << std::endl;
#endif /* HRVO_OUTPUT_TIME_AND_POSITIONS */

		simulator.doStep();
	}
	while (!simulator.haveReachedGoals());

	outfile.close();
	return 0;
}