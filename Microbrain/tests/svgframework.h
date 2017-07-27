#ifndef SVGFRAMEWORK_H
#define SVGFRAMEWORK_H

#include <stdio.h>

class SVGOutput
{
	public:
		SVGOutput(const char *outputFileName, float bottom_x, float left_y, float width, float height);
		~SVGOutput();

		void drawPoint(float x, float y);
		void drawCircle(float cx, float cy, float radius);
		void drawSegment(float x1, float y1, float x2, float y2, bool drawExtensions = false);
		void drawPolygon(const float xs[], const float ys[], unsigned int count);
		void drawCapsule(float x1, float y1, float x2, float y2, float radius);

	private:
		FILE *outputStream;
};

#endif
