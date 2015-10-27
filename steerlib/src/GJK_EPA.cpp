/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
#include <limits>


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool collision = gjk(_shapeA, _shapeB, simplex);

	if(collision)
		epa(return_penetration_depth, return_penetration_vector,simplex,_shapeA, _shapeB);
	/*else{
		return_penetration_depth = 0;
		return_penetration_vector.zero();
	}*/	
	
    return collision;
}

bool SteerLib::GJK_EPA::gjk(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex) {
	Util::Vector centerA = getCenter(_shapeA);
	Util::Vector centerB = getCenter(_shapeB);
	Util::Vector dir(1,0,0);

	simplex.push_back((getFarPoint(_shapeA, dir) - getFarPoint(_shapeB, -dir)));
	dir = -dir;
	
	while(true){
		float dotProduct = 0;
		simplex.push_back((getFarPoint(_shapeA, dir) - getFarPoint(_shapeB, -dir)));

		for(int i = 0; i < 3; i++)
			dotProduct = simplex.back()[i] * dir[i];

		if(dotProduct <= 0) 
			return false;
		else if (checkOrigin(simplex, dir))
			return true;
		}
	return false;
}

void SteerLib::GJK_EPA::epa(float& return_penetration_depth, Util::Vector& return_penetration_vector, std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	while(true)
	{
		float distance = std::numeric_limits<float>::max();
		Util::Vector normal;
		int index;
		getClosestEdge(simplex, distance, normal, index);
		
		Util::Vector help = (getFarPoint(_shapeA, normal) - getFarPoint(_shapeB, -normal));
		float d = dotProd(help, normal);

		if(d - distance < 0.00001)
		{
			return_penetration_vector = normal;
			return_penetration_depth = d;
			return;
		}
		else{
			simplex.insert(simplex.begin()+index, help);
		}

	}
}
void SteerLib::GJK_EPA::getClosestEdge(std::vector<Util::Vector> simplex, float& distance, Util::Vector& normal, int& index)
{
	for(int i = 0; i < simplex.size(); i++){
		int j = (i + 1 == simplex.size()) ? 0 : i + 1;

		Util::Vector a = simplex[i];
		Util::Vector b = simplex[j];
		Util::Vector e = b - a;
		Util::Vector n = (a*dotProd(e, e) - e*dotProd(e,a));
		n = Util::normalize(n);

		float d = dotProd(n, a);
		if(d < distance)
		{
			distance = d;
			normal = n;
			index = j;
		}
	}
}
bool SteerLib::GJK_EPA::checkOrigin(std::vector<Util::Vector>& simplex, Util::Vector& dir){
	Util::Vector pointA = simplex.back();
	Util::Vector aOrigin = -pointA;
	
	if(simplex.size() == 3){
		Util::Vector pointB = simplex[0];
		Util::Vector pointC = simplex[1];

		Util::Vector edgeAB = pointB - pointA;
		Util::Vector edgeAC = pointC - pointA;

		Util::Vector perpAB = (edgeAB*dotProd(edgeAB, edgeAC) - edgeAC*dotProd(edgeAB,edgeAB));
		Util::Vector perpAC = (edgeAC*dotProd(edgeAC, edgeAB) - edgeAB*dotProd(edgeAC,edgeAC));

		if(dotProd(perpAB, aOrigin) > 0){
			simplex.erase(simplex.begin() + 1);
			dir = perpAB;
		}
		else if(dotProd(perpAC, aOrigin) > 0){
			simplex.erase(simplex.begin());
			dir = perpAC;
		}
		else return true;
	}
	else{
		Util::Vector pointB = simplex[0];
		Util::Vector edgeAB = pointB - pointA;
		Util::Vector perpAB = (aOrigin*dotProd(edgeAB, edgeAB) - edgeAB*dotProd(edgeAB, aOrigin));
		dir = perpAB;
	}
	return false;
	
}

Util::Vector SteerLib::GJK_EPA::getFarPoint(const std::vector<Util::Vector>& shape, const Util::Vector& dir){

	Util::Vector farPoint(0,0,0);
	float farDistance = 0;
	float farIndex = 0;

	for(int i = 0; i < shape.size(); i++){
		float checkFar = dotProd(shape[i], dir);
		if (checkFar > farDistance){
			farDistance = checkFar;
			farIndex = i;
		}
	}
	farPoint[0] = shape[farIndex][0];
	farPoint[1] = shape[farIndex][1];
	farPoint[2] = shape[farIndex][2];

	return farPoint;
}


Util::Vector SteerLib::GJK_EPA::getCenter(const std::vector<Util::Vector>& shape){
	Util::Vector center(0,0,0);

	for(int i = 0; i < shape.size(); i++){
		center += shape[i];
	}

	return center/(float)shape.size();
}

float SteerLib::GJK_EPA::dotProd(Util::Vector A, Util::Vector B)
{
	return (A[0] * B[0]) + (A[1] * B[1]) + (A[2] * B[2]);
}	
