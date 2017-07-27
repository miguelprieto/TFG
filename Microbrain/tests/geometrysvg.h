#ifndef GEOMETRYSVG_H
#define GEOMETRYSVG_H

#include <errno.h>
#include <sys/stat.h>

#include "svgframework.h"
#include "testframework.h"

#include "geometry.h"

// Funzione che disegna in SVG il parametro in input, qualora esso sia una
// figura geometrica. Per i parametri di tipo sconosciuto (ovvero quelli per cui
// non è presente una specializzazione) non viene disegnato nulla
template <typename T>
void drawArgument(SVGOutput *svgOutput, const T &value)
{
}

template <>
void drawArgument<Point>(SVGOutput *svgOutput, const Point &value)
{
	svgOutput->drawPoint(value.x, value.y);
}

template <>
void drawArgument<Circle>(SVGOutput *svgOutput, const Circle &value)
{
	svgOutput->drawCircle(value.center().x, value.center().y, value.radius());
}

template <>
void drawArgument<Line>(SVGOutput *svgOutput, const Line &value)
{
	svgOutput->drawSegment(value.pointA().x, value.pointA().y, value.pointB().x, value.pointB().y, true);
}

template <>
void drawArgument<Segment>(SVGOutput *svgOutput, const Segment &value)
{
	svgOutput->drawSegment(value.pointA().x, value.pointA().y, value.pointB().x, value.pointB().y, false);
}

template <>
void drawArgument<Capsule>(SVGOutput *svgOutput, const Capsule &value)
{
	svgOutput->drawCapsule(value.pointA().x, value.pointA().y, value.pointB().x, value.pointB().y, value.radius());
}

template <>
void drawArgument<Polygon>(SVGOutput *svgOutput, const Polygon &value)
{
	float *xs = new float[value.vertexCount()];
	float *ys = new float[value.vertexCount()];

	for (unsigned int i = 0; i < value.vertexCount(); i++)
	{
		xs[i] = value.vertexAtIndex(i).x;
		ys[i] = value.vertexAtIndex(i).y;
	}

	svgOutput->drawPolygon(xs, ys, value.vertexCount());

	delete[] xs;
	delete[] ys;
}

template <>
void drawArgument<IntersectionData>(SVGOutput *svgOutput, const IntersectionData &value)
{
	for (unsigned int i = 0; i < value.intersectionCount() && value.intersectionCount() != UINT_MAX; i++)
		svgOutput->drawPoint(value.intersectionPoint(i).x, value.intersectionPoint(i).y);
}

// Classe decorator di TestWithDataRunner<...> che crea automaticamente un file
// SVG con le figure geometriche presenti in ogni caso testato.
template <typename ...Args>
class TestWithDataRunnerSvgDecorator : public TestWithDataRunner<Args...>
{
	public:
		TestWithDataRunnerSvgDecorator(const char *subDirectoryName, const TestWithDataRunner<Args...> &realTestRunner,
			float bottom_x, float left_y, float width, float height)
		: subDirectoryName(subDirectoryName), bottom_x(bottom_x), left_y(left_y), width(width), height(height), realTestRunner(realTestRunner)
		{
		}

		void runOnData(const char *data_description, const Args&... data) const
		{
			char *temp;

			asprintf(&temp, "%s.figures", program_invocation_short_name);
			mkdir(temp, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			free(temp);

			asprintf(&temp, "%s.figures/%s", program_invocation_short_name, subDirectoryName);
			mkdir(temp, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			free(temp);

			asprintf(&temp, "%s.figures/%s/%s.svg", program_invocation_short_name, subDirectoryName, data_description);
			SVGOutput *svgOutput = new SVGOutput(temp, bottom_x, left_y, width, height);
			drawArguments(svgOutput, data...);
			delete svgOutput;
			//fprintf(stderr, "Generato %s\n", temp);
			free(temp);

			realTestRunner.runOnData(data_description, data...);
		}

		// Funzione ricorsiva che richiama drawArgument() per ogni argumento
		template <typename First, typename ...OtherArgs>
		void drawArguments(SVGOutput *svgOutput, const First &head, const OtherArgs&... tail) const
		{
			drawArgument(svgOutput, head);
			drawArguments(svgOutput, tail...);
		}

		// Caso base:
		void drawArguments(SVGOutput *svgOutput) const
		{
		}

	private:
		const char *subDirectoryName;
		float bottom_x, left_y, width, height;
		const TestWithDataRunner<Args...> &realTestRunner;
};

#endif
