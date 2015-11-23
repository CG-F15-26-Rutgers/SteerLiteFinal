/*!
*
* \author VaHiD AzIzI
*
*/

#include <cmath>
#include <algorithm>
#include <stack>
#include <queue>
#include <cmath>
#include "obstacles/GJK_EPA.h"

#define PRINT_TRIANGLES true

SteerLib::GJK_EPA::GJK_EPA()
{
}

/************************************************/
/****************** MATH STUFF ******************/
/************************************************/
float SteerLib::GJK_EPA::DotProduct(Util::Vector A, Util::Vector B)
{
	return (A.x * B.x) + (A.y * B.y) + (A.z * B.z);
}

Util::Vector SteerLib::GJK_EPA::TripleProduct(Util::Vector A, Util::Vector B, Util::Vector C) {
	// Formula: (A  x  B) x C =   B(C  dot  A) -  A(C  dot  B)
	// Example: (AB x A0) x AB = A0(AB dot AB) - AB(AB dot A0)

	float CdotA = DotProduct(C, A);
	float CdotB = DotProduct(C, B);

	return B*CdotA - A*CdotB;
}

Util::Vector SteerLib::GJK_EPA::Midpoint(Util::Vector p1, Util::Vector p2)
{
	double x = (p1.x + p2.x) / 2;
	double z = (p1.z + p2.z) / 2;

	return Util::Vector(x, 0, z);
}

double SteerLib::GJK_EPA::CheckDirection(std::vector<Util::Vector> shape)
{
	double sum = 0;
	for (int i = 0; i < shape.size(); i++) {
		if (i == shape.size() - 1) {
			// last and first points
			sum += (shape[0].x - shape[i].x) * (shape[0].z - shape[i].z);
		}
		else {
			sum += (shape[i + 1].x - shape[i].x) * (shape[i + 1].z + shape[i].z);
		}
	}

	return sum;
}

/*****************************************/
/****************** GJK ******************/
/*****************************************/
int SteerLib::GJK_EPA::GetFarthest(const std::vector<Util::Vector>& shape, Util::Vector d)
{
	double maxdot = d * shape[0];
	double dotproduct;
	int farthest = 0;

	for (int i = 0; i < shape.size(); i++) {
		dotproduct = d * shape[i];

		if (dotproduct > maxdot) {
			maxdot = dotproduct;
			farthest = i;
		}
	}

	return farthest;
}

Util::Vector SteerLib::GJK_EPA::Support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector d)
{
	Util::Vector A = _shapeA[GetFarthest(_shapeA, d)];
	Util::Vector B = _shapeB[GetFarthest(_shapeB, (d * -1))];

	return A - B;
}

void SteerLib::GJK_EPA::NegateDirection(Util::Vector& d)
{
	d = d * -1;
}

bool SteerLib::GJK_EPA::Origins(std::vector<Util::Vector>& _simplex, Util::Vector& d)
{
	Util::Vector A = _simplex.back();
	Util::Vector A0 = -A;

	if (_simplex.size() == 3) {
		// Triangle case

		// Get points from simplex
		Util::Vector B = _simplex.at(1);
		Util::Vector C = _simplex.at(0);

		// Get edges
		Util::Vector AB = B - A;
		Util::Vector AC = C - A;

		// Reset direction
		d = Util::Vector(AB.z, AB.y, AB.x * -1);

		if (d * C > 0)
			NegateDirection(d);

		if (d * A0 > 0) {
			_simplex.erase(_simplex.begin() + 0);
			
			return false;
		}

		d = Util::Vector(AC.z, AC.y, AC.x * -1);

		if (d * A0 > 0) {
			_simplex.erase(_simplex.begin() + 1);

			return false;
		}
	
		// We found the origin!
		return true;
	}
	else {
		// Line segment case
		Util::Vector B = _simplex.at(0);
		Util::Vector AB = B - A;

		d = Util::Vector(AB.z, AB.y, AB.x * -1);

		if (d * A0 > 0)
			NegateDirection(d);
	}

	return false;
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex)
{
	Util::Vector d(1, 0, -1);
	_simplex.push_back(Support(_shapeA, _shapeB, d));
	d = d * -1;

	while (true) {
		_simplex.push_back(Support(_shapeA, _shapeB, d));

		if (_simplex.back() * d <= 0) {
			return false;
		}
		else {
			if (Origins(_simplex, d))
				return true;
		}
	}

	return false;
}


/*****************************************/
/****************** EPA ******************/
/*****************************************/
// TODO

/***********************************************************/
/****************** POLYGON TRIANGULATION ******************/
/********************* Via Ear Clipping ********************/
/***********************************************************/
void SteerLib::GJK_EPA::RemoveEdge(std::vector<struct edges>& edges, Util::Vector point)
{
	Util::Vector n1, n2;
	for (int i = 0; i < edges.size(); i++) {
		if (edges[i].point == point) {
			n1 = edges[i].neighbor1;
			n2 = edges[i].neighbor2;

			edges.erase(edges.begin() + i);
		}

		if (edges[i].neighbor1 == point) {
			if (edges[i].point == n1) {
				edges[i].neighbor1 = n2;
			}
			else if (edges[i].point == n2) {
				edges[i].neighbor1 = n1;
			}
		}
		else if (edges[i].neighbor2 == point) {
			if (edges[i].point == n1) {
				edges[i].neighbor2 = n2;
			}
			else if (edges[i].point == n2) {
				edges[i].neighbor2 = n1;
			}
		}
	}
}

void SteerLib::GJK_EPA::RemovePoint(std::vector<Util::Vector>& shape, Util::Vector point)
{
	for (int i = 0; i < shape.size(); i++) {
		if (shape[i] == point) {
			shape.erase(shape.begin() + i);
		}
	}
}

bool SteerLib::GJK_EPA::EarCheck(std::queue<Util::Vector> ears, Util::Vector ear)
{
	while (!ears.empty()) {
		if (ears.front() == ear)
			return true;

		ears.pop();
	}

	return false;
}

void SteerLib::GJK_EPA::UpdateEars(std::queue<Util::Vector>& ears, std::vector<struct edges> edges, std::vector<Util::Vector> shape)
{
	for (int i = 0; i < shape.size(); i++) {
		Util::Vector curr = shape[i];
		Util::Vector n1, n2, testpoint;

		for (int j = 0; j < edges.size(); j++) {
			if (edges[j].point == curr) {
				n1 = edges[j].neighbor1;
				n2 = edges[j].neighbor2;

				break;
			}
		}

		// Get midpoint of the two neighbors
		testpoint = Midpoint(n1, n2);

		// Test the midpoint 
		// If it lies within the polygon, it's good
		bool result = false;
		int k, j;
		for (k = 0, j = shape.size() - 1; k < shape.size(); j = k++) {
			if ((shape[k].z > testpoint.z) != (shape[j].z > testpoint.z) && (testpoint.x < (shape[j].x - shape[k].x) * (testpoint.z - shape[k].z) / (shape[j].z - shape[k].z) + shape[k].x)) {
				result = !result;
			}
		}

		// TODO ONLY IF THEY'RE GONNA HAVE OTHER TEST CASES
		// Need to do check if point is on an edge

		// If midpoint is a point on shape, it's not an ear
		if (std::find(shape.begin(), shape.end(), testpoint) != shape.end())
			result = false;

		if (result) {
			if(!EarCheck(ears, shape[i]))
				ears.push(shape[i]);
		}
	}
}

void SteerLib::GJK_EPA::FindEars(std::vector<struct triangle>& triangles, std::vector<Util::Vector> shape, std::vector<struct edges> edges)
{
	std::queue<Util::Vector> ears;

	// Get initial ears
	UpdateEars(ears, edges, shape);

	// Loop while there are still more than 3 points in the shape
	while(shape.size() > 3) {
		Util::Vector ear = ears.front();
		Util::Vector n1, n2;
		
		// Get Neighbors of ear
		for (int i = 0; i < edges.size(); i++) {
			if (edges[i].point == ear) {
				n1 = edges[i].neighbor1;
				n2 = edges[i].neighbor2;
				break;
			}
		}

		// Make triangle out of ear and its two neighbors
		struct triangle t = { ear, n1, n2 };
		triangles.push_back(t);

		// Remove edge that has the ear
		// Points that have the ear as its neighbor are neighbored
		RemoveEdge(edges, ear);
		
		// Remove ear from shape
		RemovePoint(shape, ear);

		// Remove ear from queue
		ears.pop();		

		// Update queue since new ears could have been found
		UpdateEars(ears, edges, shape);
	}

	// Add the last three points as a triangle
	struct triangle t = { shape[0], shape[1], shape[2] };
	triangles.push_back(t);

	if (PRINT_TRIANGLES) {
		for (int i = 0; i < triangles.size(); i++) {
			printf("Triangle %d\n", i + 1);
			printf("<%f, %f, %f> - ", triangles[i].p1.x, triangles[i].p1.y, triangles[i].p1.z);
			printf("<%f, %f, %f> - ", triangles[i].p2.x, triangles[i].p2.y, triangles[i].p2.z);
			printf("<%f, %f, %f>\n", triangles[i].p3.x, triangles[i].p3.y, triangles[i].p3.z);
			printf("**************\n");
		}
	}
}

void SteerLib::GJK_EPA::GetEdges(std::vector<struct triangle>& triangles, const std::vector<Util::Vector>& _shape)
{
	// Since points are entered clockwise, we reverse it
	std::vector <Util::Vector> shape = _shape;
	std::reverse(shape.begin(), shape.end());

	// Pair edges and put into edge structs
	std::vector<struct edges> edges;
	for (int i = 0; i < shape.size(); i++) {
		struct edges temp;
		Util::Vector point = shape[i];
		Util::Vector neighbor1;
		Util::Vector neighbor2;

		if (i == 0) {
			neighbor1 = shape[shape.size() - 1];
			neighbor2 = shape[i + 1];
		}
		else if (i == shape.size() - 1) {
			neighbor1 = shape[i - 1];
			neighbor2 = shape[0];
		}
		else {
			neighbor1 = shape[i - 1];
			neighbor2 = shape[i + 1];
		}

		temp.point = point;
		temp.neighbor1 = neighbor1;
		temp.neighbor2 = neighbor2;

		edges.push_back(temp);
	}

	// Get initial set of ears
	FindEars(triangles, shape, edges);
}

bool SteerLib::GJK_EPA::Triangulate(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<struct triangle> trianglesA, trianglesB;
	// Pairs edges from original polygon together and also gets the triangulated points
	GetEdges(trianglesA, _shapeA);
	GetEdges(trianglesB, _shapeB);
		
	std::vector<Util::Vector> _simplex;
	for (int i = 0; i < trianglesA.size(); i++) {
		// Get points of a triangle from shapeA
		std::vector<Util::Vector> a;
		a.push_back(trianglesA[i].p1);
		a.push_back(trianglesA[i].p2);
		a.push_back(trianglesA[i].p3);

		for (int j = 0; j < trianglesB.size(); j++) {
			_simplex.clear();

			// Get points of a triangle from shapeB
			std::vector<Util::Vector> b;
			b.push_back(trianglesB[j].p1);
			b.push_back(trianglesB[j].p2);
			b.push_back(trianglesB[j].p3);
			
			// Perform GJK on every pair of trianlges from the two polygons
			if (GJK(a, b, _simplex))
				return true;
		}
	}

	return false;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;
	return Triangulate(_shapeA, _shapeB); 
}
