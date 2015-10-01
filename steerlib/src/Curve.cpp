//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
        for(int i = 0; i < controlPoints.size()-1; i++)
	{
	  int indMinElement = i;
	  for(int j = i+1; j < controlPoints.size(); j++)
	  {
	    if((controlPoints[j]).time < (controlPoints[indMinElement]).time)
	    {
	      int indMinElement = j;
	    }
	  }
	  std::swap(controlPoints[i], controlPoints[indMinElement]);
	}

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	return controlPoints.size() > 1;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
       int i = 0;
       for(; i < controlPoints.size() && (controlPoints[i]).time <= time; i++);
       if(i < controlPoints.size())
       {
	 nextPoint = i;
	 return true;
       }
       return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[nextPoint].time;
	intervalTime = intervalTime-time;
	//intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;

	//how do you normalize data though?

	//using the answer from this http://stats.stackexchange.com/questions/70801/how-to-normalize-data-to-0-1-range
	//normalizedTime = (time - min(time))/((max(time)) - (min(time))

	//max index = size - 1; min index = 0
	normalTime = (time - controlPoints[0].time)/((controlPoints[controlPoints.size()-1].time) - (controlPoints[0].time));

	// Calculate position at t = time on Hermite curve
	//do we use the param time, intervalTime, or normalTime? 
	//Like what was the point of finding intervalTime?

	//blending functions used to determine h1 thru h4
	float h1, h2, h3, h4;
	float usefulTime = intervalTime; //whatever time we actually end up using 
	//h1 = 2t^3 - 3t^2 + 1
	h1 = (2*pow(usefulTime, 3)) - (3*pow(usefulTime,2)) + 1;
	//h2 = -2t^3 + 3t^2
	h2 = (-2*pow(usefulTime,3)) + (3*pow(usefulTime,2));
	//h3 = t^3 - 2t^2 + t
	h3 = pow(usefulTime,3) - (2*pow(usefulTime,2)) + usefulTime;
	//h4 = t^3 - t^2
	h4 = pow(usefulTime,3) - pow(usefulTime,2);

	//h1,h2 correlate with position; h3,h4 correlate with tangent
	newPosition = h1*controlPoints[nextPoint-1].position + h2*controlPoints[nextPoint].position + h3*controlPoints[nextPoint-1].tangent + h4*controlPoints[nextPoint].tangent;
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
}
