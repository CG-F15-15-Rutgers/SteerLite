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
	// Robustness: make sure there is at least two control point: start and end points
		if(!checkRobust())
			return false;
		else{
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
			float t0 = controlPoints.front().time;
			float tFinal = controlPoints.back().time;
			Point back = controlPoints.front().position;
			Point curr = controlPoints.front().position;

			for(t = window; t <= tFinal; t += window){
				if(t > tFinal - window) //final point
					curr = controlPoints.back().position;
				else{
	//Calculate the next point, draw the line, update the tracer
					calculatePoint(curr, t);
					drawLine(back, curr, curveColor, curveThickness);
					back = curr;
				}
			}
		}
	
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
	Util::Vector vect;
	float normalTime, intervalTime;
	// Calculate time interval, and normal time required for later curve calculations
	//intervalTime = controlPoints[nextPoint].time;
	//intervalTime = intervalTime-time;
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	normalTime = (time - controlPoints[nextPoint-1].time)/(intervalTime);

	// Calculate position at t = time on Hermite curve
	//do we use the param time, intervalTime, or normalTime? 
	//Like what was the point of finding intervalTime?

	//blending functions used to determine h1 thru h4
	float h1, h2, h3, h4;
	float tcube, tsquare;
	tcube = normalTime*normalTime*normalTime;
	tsquare = normalTime*normalTime;
	//h1 = 2t^3 - 3t^2 + 1
	h1 = (2*tcube) - (3*tsquare) + 1;
	//h2 = -2t^3 + 3t^2
	h2 = (-2*tcube) + (3*tsquare);
	//h3 = t^3 - 2t^2 + t
	h3 = tcube - (2*tsquare) + normalTime;
	//h4 = t^3 - t^2
	h4 = tcube - tsquare;

	Point p1 = h1 * controlPoints[nextPoint-1].position;
	Point p2 = h2 * controlPoints[nextPoint].position;
	Point p3, p4;

	vect = controlPoints[nextPoint-1].tangent * (h3 * intervalTime);
	p3 = Point(vect[0],vect[1], vect[2]);

	vect = controlPoints[nextPoint].tangent * (h4 * intervalTime);
	p4 = Point(vect[0],vect[1],vect[2]);

	//h1,h2 correlate with position; h3,h4 correlate with tangent
	//newPosition = (h1*controlPoints[nextPoint-1].position) + (h2*controlPoints[nextPoint].position) + (h3*controlPoints[nextPoint-1].tangent) + (h4*controlPoints[nextPoint].tangent);
	newPosition = p1 + p2 + p3 + p4;
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition, curr, back;
	float t0,t1, normalTime, intervalTime;
	Vector v1, v2;

	// Calculate time interval, and normal time required for later curve calculations
	t0 = controlPoints[nextPoint-1].time;
	t1 = controlPoints[nextPoint].time;

	intervalTime = t1 - t0;
	normalTime = (time - t0) / intervalTime;

	// Calculate position at t = time on Catmull-Rom curve
	
	if(nextPoint == 1) //first point
	{
	v1 = (controlPoints[nextPoint].position - controlPoints[nextPoint-1].position)/intervalTime;
	v2 = (controlPoints[nextPoint+1].position - controlPoints[nextPoint-1].position)/(controlPoints[nextPoint+1].time - t0);
	}
	else if (nextPoint == controlPoints.size()-1) //final point
	{
	v1 = (controlPoints[nextPoint].position - controlPoints[nextPoint-2].position)/(t1 - controlPoints([nextPoint-2].time);
	v2 = (controlPoints[nextPoint].position - controlPoints[nextPoint-1].position)/intervalTime;
	}
	else //inbetween points
	{
	v1 = (controlPoints[nextPoint].position - controlPoints[nextPoint-2].position)/(t1 - controlPoints([nextPoint-2].time);
	v2 = (controlPoints[nextPoint+1].position - controlPoints[nextPoint-1].position)/(controlPoints[nextPoint+1].time - t0);
	}

	float tcube, tsquare;
	tcube = normalTime*normalTime*normalTime;
	tsquare = normalTime*normalTime;

	//Blending Functions
	h1 = (2*tcube) - (3*tsquare) + 1;
	h2 = (-2*tcube) + (3*tsquare);
	h3 = tcube - (2*tsquare) + normalTime;
	h4 = tcube - tsquare;

	newPosition = h1*controlPoints[nextPoint-1].position + h2*controlPoints[nextPoint].position
			+ intervalTime*h3*v1 + intervalTime*h4*v2;

	// Return result
	return newPosition;
}
