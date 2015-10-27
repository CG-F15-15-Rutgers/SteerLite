/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}

bool SteerLib::GJK_EPA::gjk(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::std::vector<Util::Vector>& simplex) {

}




