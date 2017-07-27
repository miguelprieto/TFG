#include "svgframework.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#define SVG_SHAPE_STYLE "fill:pink;stroke:blue;stroke-width:.5;fill-opacity:0.5;stroke-opacity:0.9"
#define SVG_LINE_STYLE "stroke:gray;stroke-width:.2;stroke-opacity:0.9"
#define SVG_LINEEXT_STYLE "stroke:gray;stroke-width:.2;stroke-opacity:0.4"

SVGOutput::SVGOutput(const char *outputFileName, float bottom_x, float left_y, float width, float height)
{
	outputStream = fopen(outputFileName, "wt");
	if (outputStream == nullptr)
	{
		perror(outputFileName);
		_exit(EXIT_FAILURE);
	}

	const float scale = 20;
	fprintf(outputStream, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%f\" height=\"%f\" transform=\"translate(0, %f) scale(%f, -%f) translate(%f, %f)\">\n",
		width*scale, height*scale, height*scale, scale, scale, -bottom_x, -left_y);

	// Draw + at (0,0)
	drawSegment(-5, 0, +5, 0);
	drawSegment(0, -5, 0, +5);
}

SVGOutput::~SVGOutput()
{
	fprintf(outputStream, "</svg>\n");
	fclose(outputStream);
}

void SVGOutput::drawPoint(float x, float y)
{
	fprintf(outputStream, "<circle cx=\"%f\" cy=\"%f\" r=\".1\" fill=\"black\" stroke=\"black\" stroke-width=\".5\" />\n", x, y);
}

void SVGOutput::drawCircle(float cx, float cy, float radius)
{
	fprintf(outputStream, "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" style=\"%s\" />\n", cx, cy, radius, SVG_SHAPE_STYLE);
}

void SVGOutput::drawSegment(float x1, float y1, float x2, float y2, bool drawExtensions)
{
	fprintf(outputStream, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"%s\" />\n", x1, y1, x2, y2, SVG_LINE_STYLE);

	if (drawExtensions)
	{
		float dx = (x2 - x1) / 10;
		float dy = (y2 - y1) / 10;

		for (unsigned int i = 0; i < 3; i++)
		{
			float dx1 = (2*i + 1) * dx;
			float dy1 = (2*i + 1) * dy;
			float dx2 = (2*i + 2) * dx;
			float dy2 = (2*i + 2) * dy;
			fprintf(outputStream, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"%s\" />\n", x1 - dx1, y1 - dy1, x1 - dx2, y1 - dy2, SVG_LINEEXT_STYLE);
			fprintf(outputStream, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"%s\" />\n", x2 + dx1, y2 + dy1, x2 + dx2, y2 + dy2, SVG_LINEEXT_STYLE);
		}
	}
}

void SVGOutput::drawPolygon(const float xs[], const float ys[], unsigned int count)
{
	fprintf(outputStream, "<polygon style=\"%s\" points=\"", SVG_SHAPE_STYLE);

	for (unsigned int i = 0; i < count; i++)
	{
		if (i != 0)
			fprintf(outputStream, " ");
		fprintf(outputStream, "%f,%f", xs[i], ys[i]);
	}

	fprintf(outputStream, "\" />\n");
}

void SVGOutput::drawCapsule(float x1, float y1, float x2, float y2, float radius)
{
	fprintf(outputStream, "<path style=\"%s\" d=\"", SVG_SHAPE_STYLE);

	const double dir = M_PI_2 + atan2(y2 - y1, x2 - x1);
	const double dx = cos(dir) * radius;
	const double dy = sin(dir) * radius;

	fprintf(outputStream, "M %f %f ", x1 + dx, y1 + dy);
	fprintf(outputStream, "A %f %f %f 1 1 %f %f ", radius, radius, (dir+1)*180/M_PI, x1 - dx, y1 - dy);

	fprintf(outputStream, "L %f %f ", x2 - dx, y2 - dy);
	fprintf(outputStream, "A %f %f %f 1 1 %f %f ", radius, radius, (dir+1)*180/M_PI, x2 + dx, y2 + dy);

	fprintf(outputStream, "Z\" />\n");
}
