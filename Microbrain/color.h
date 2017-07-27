/*
 * color.h
 */

#ifndef __COLOR_H
#define __COLOR_H

#include "routing.h"

typedef enum
{
	YELLOW,	// 0
	BLUE	// 1
} color_t;

extern color_t color;

template <typename T>
class ColorDependentValue
{
	public:
		ColorDependentValue(const T &valueIfYellow, const T &valueIfBlue)
		: valueIfYellow(valueIfYellow), valueIfBlue(valueIfBlue)
		{
		}

		const T& getValue() const
		{
			if (color == YELLOW)
				return valueIfYellow;
			else
				return valueIfBlue;
		}

		operator const T&() const
		{
			return getValue();
		}

		const T operator->() const
		{
			return getValue();
		}

	private:
		const T valueIfYellow, valueIfBlue;
};

inline ColorDependentValue<double> symmXifBlue(double valueIfYellow)
{
	return ColorDependentValue<double>(valueIfYellow, 3000 - valueIfYellow);
}

inline ColorDependentValue<int> symmTifBlue(int valueIfYellow)
{
	int symmT = 180 - valueIfYellow;
	if (symmT < -180)
		symmT += 360;
	else if (symmT > 180)
		symmT -= 360;
	return ColorDependentValue<int>(valueIfYellow, symmT);
}

inline ColorDependentValue<GraphNode*> symmNodeifBlue(GraphNodeWithSymmetric *nodeIfYellow)
{
	return ColorDependentValue<GraphNode*>(nodeIfYellow, nodeIfYellow->symmetric);
}

#endif
