#include "geometrysvg.h"
#include "testframework.h"

#include "geometry.h"

// --- normalize_angle_test ---
static void normalize_angle_test_data(const TestWithDataRunner<double, double> &testRunner)
{
	double oneDeg = PI / 180;

	testRunner.runOnData("0", 0, 0);
	testRunner.runOnData("+90", HALF_PI, HALF_PI);
	testRunner.runOnData("-90", -HALF_PI, -HALF_PI);

	testRunner.runOnData("+179", +PI-oneDeg, +PI-oneDeg);
	testRunner.runOnData("+180", +PI, +PI);
	testRunner.runOnData("+181", +PI+oneDeg, -PI+oneDeg);

	testRunner.runOnData("-179", -PI+oneDeg, -PI+oneDeg);
	testRunner.runOnData("-180", -PI, -PI);
	testRunner.runOnData("-181", -PI-oneDeg, +PI-oneDeg);

	testRunner.runOnData("+359", +TWO_PI-oneDeg, -oneDeg);
	testRunner.runOnData("+360", +TWO_PI, 0);
	testRunner.runOnData("+361", +TWO_PI+oneDeg, +oneDeg);

	testRunner.runOnData("-359", -TWO_PI+oneDeg, +oneDeg);
	testRunner.runOnData("-360", -TWO_PI, 0);
	testRunner.runOnData("-361", -TWO_PI-oneDeg, -oneDeg);

	testRunner.runOnData("+539", +TWO_PI+PI-oneDeg, +PI-oneDeg);
	testRunner.runOnData("+540", +TWO_PI+PI, +PI);
	//testRunner.runOnData("+541", +TWO_PI+PI+oneDeg, -PI+oneDeg);

	testRunner.runOnData("-539", -TWO_PI-PI+oneDeg, -PI+oneDeg);
	testRunner.runOnData("-540", -TWO_PI-PI, -PI);
	//testRunner.runOnData("-541", -TWO_PI-PI-oneDeg, +PI-oneDeg);
}

static void normalize_angle_test(double input, double expectedOutput)
{
	assert(fabs(normalize_angle(input) - expectedOutput) < .0001);
}

REGISTER_TEST_WITH_DATA(normalize_angle_test, normalize_angle_test_data, double, double)

// --- circle_containsPoint_test ---
static void circle_containsPoint_test_data(const TestWithDataRunner<Circle, Point, bool> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Circle, Point, bool> svgTestRunner(
		"circle_containsPoint_test", testRunner,
		-6, -6, 30, 30);

	Circle testCircle(Point(4, 4), sqrt(32));

	svgTestRunner.runOnData("center", testCircle, Point(4, 4), true);
	svgTestRunner.runOnData("near_border_inside_1", testCircle, Point(7, 7), true);
	svgTestRunner.runOnData("near_border_inside_2", testCircle, Point(8, 7), true);
	svgTestRunner.runOnData("near_border_inside_3", testCircle, Point(7, 8), true);
	svgTestRunner.runOnData("on_border", testCircle, Point(8, 8), true);
	svgTestRunner.runOnData("near_border_outside_1", testCircle, Point(9, 8), false);
	svgTestRunner.runOnData("near_border_outside_2", testCircle, Point(8, 9), false);
	svgTestRunner.runOnData("near_border_outside_3", testCircle, Point(9, 9), false);
	svgTestRunner.runOnData("far", testCircle, Point(15, 13), false);
}

static void circle_containsPoint_test(Circle circle, Point point, bool expectedOutput)
{
	assert(circle.containsPoint(point) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(circle_containsPoint_test, circle_containsPoint_test_data, Circle, Point, bool)

// --- circle_intersectCircle_test ---
static void circle_intersectCircle_test_data(const TestWithDataRunner<Circle, Circle, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Circle, Circle, IntersectionData> svgTestRunner(
		"circle_intersectCircle_test", testRunner,
		-6, -6, 30, 30);

	Circle testCircle(Point(4, 4), sqrt(32));

	svgTestRunner.runOnData("non_intersecting", testCircle, Circle(Point(12, 12), 4), IntersectionData());
	svgTestRunner.runOnData("same_center_smaller_radius", testCircle, Circle(Point(4, 4), 4), IntersectionData().setAreasIntersectFlag());
	svgTestRunner.runOnData("same_center_same_radius", testCircle, Circle(Point(4, 4), sqrt(32)), IntersectionData().setInfiniteIntersectionPoints());
	svgTestRunner.runOnData("same_center_bigger_radius", testCircle, Circle(Point(4, 4), 8), IntersectionData().setAreasIntersectFlag());
	svgTestRunner.runOnData("tangent_sut_inside_arg", testCircle, Circle(Point(6, 6), sqrt(72)), IntersectionData().addIntersectionPoint(Point(0, 0)));
	svgTestRunner.runOnData("tangent_arg_inside_sut", testCircle, Circle(Point(6, 6), sqrt(8)), IntersectionData().addIntersectionPoint(Point(8, 8)));

	svgTestRunner.runOnData("tangent_outside", testCircle, Circle(Point(10, 10), sqrt(8)), IntersectionData().addIntersectionPoint(Point(8, 8)));

	const double a1 = 7.011180558268442, a2 = 8.788819441731558;
	svgTestRunner.runOnData("intersecting_near", testCircle, Circle(Point(9, 9), 2), IntersectionData().addIntersectionPoint(Point(a1, a2)).addIntersectionPoint(Point(a2, a1)));

	const double b1 = -1.557189138830738, b2 = 5.057189138830739;
	svgTestRunner.runOnData("intersecting_far", testCircle, Circle(Point(8, 8), 10), IntersectionData().addIntersectionPoint(Point(b1, b2)).addIntersectionPoint(Point(b2, b1)));
}

static void circle_intersectCircle_test(Circle sut, Circle arg, IntersectionData expectedOutput)
{
	assert(sut.intersect(arg) == expectedOutput);

	// this should give the same result
	assert(arg.intersect(sut) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(circle_intersectCircle_test, circle_intersectCircle_test_data, Circle, Circle, IntersectionData)

// --- line_projectPoint_test ---
static void line_projectPoint_test_data(const TestWithDataRunner<Line, Point, Point, double> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Line, Point, Point, double> svgTestRunner(
		"line_projectPoint_test", testRunner,
		-6, -6, 30, 30);

	svgTestRunner.runOnData("vert_on_line1", Line(Point(2, 2), Point(2, 8)), Point(2, 9), Point(2, 9), 0);
	svgTestRunner.runOnData("vert_external1", Line(Point(2, 2), Point(2, 8)), Point(5, 9), Point(2, 9), 3);
	svgTestRunner.runOnData("vert_on_line2", Line(Point(2, 2), Point(2, 8)), Point(2, 5), Point(2, 5), 0);
	svgTestRunner.runOnData("vert_external2", Line(Point(2, 2), Point(2, 8)), Point(5, 5), Point(2, 5), 3);
	svgTestRunner.runOnData("vert_on_line3", Line(Point(2, 2), Point(2, 8)), Point(2, 1), Point(2, 1), 0);
	svgTestRunner.runOnData("vert_external3", Line(Point(2, 2), Point(2, 8)), Point(5, 1), Point(2, 1), 3);

	svgTestRunner.runOnData("horiz_on_line1", Line(Point(2, 2), Point(8, 2)), Point(9, 2), Point(9, 2), 0);
	svgTestRunner.runOnData("horiz_external1", Line(Point(2, 2), Point(8, 2)), Point(9, 5), Point(9, 2), 3);
	svgTestRunner.runOnData("horiz_on_line2", Line(Point(2, 2), Point(8, 2)), Point(5, 2), Point(5, 2), 0);
	svgTestRunner.runOnData("horiz_external2", Line(Point(2, 2), Point(8, 2)), Point(5, 5), Point(5, 2), 3);
	svgTestRunner.runOnData("horiz_on_line3", Line(Point(2, 2), Point(8, 2)), Point(1, 2), Point(1, 2), 0);
	svgTestRunner.runOnData("horiz_external3", Line(Point(2, 2), Point(8, 2)), Point(1, 5), Point(1, 2), 3);

	svgTestRunner.runOnData("diag_on_line1", Line(Point(2, 2), Point(8, 8)), Point(9, 9), Point(9, 9), 0);
	svgTestRunner.runOnData("diag_external1", Line(Point(2, 2), Point(8, 8)), Point(8, 10), Point(9, 9), sqrt(2));
	svgTestRunner.runOnData("diag_on_line2", Line(Point(2, 2), Point(8, 8)), Point(5, 5), Point(5, 5), 0);
	svgTestRunner.runOnData("diag_external2", Line(Point(2, 2), Point(8, 8)), Point(4, 6), Point(5, 5), sqrt(2));
	svgTestRunner.runOnData("diag_on_line3", Line(Point(2, 2), Point(8, 8)), Point(1, 1), Point(1, 1), 0);
	svgTestRunner.runOnData("diag_external3", Line(Point(2, 2), Point(8, 8)), Point(0, 2), Point(1, 1), sqrt(2));
}

static void line_projectPoint_test(Line line, Point arg, Point expectedResult, double expectedDistance)
{
	assert(line.projectPoint(arg) == expectedResult);
	assert(fabs(line.pointDistance(arg) - expectedDistance) < .001);
}

REGISTER_TEST_WITH_DATA(line_projectPoint_test, line_projectPoint_test_data, Line, Point, Point, double)

// --- line_intersectCircle_test ---
static void line_intersectCircle_test_data(const TestWithDataRunner<Line, Circle, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Line, Circle, IntersectionData> svgTestRunner(
		"line_intersectCircle_test", testRunner,
		-6, -6, 30, 30);

	Line line(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("non_intersecting", line, Circle(Point(10, 1), 1), IntersectionData());
	svgTestRunner.runOnData("tangent", line, Circle(Point(10, 1), 10 / sqrt(2)), IntersectionData().addIntersectionPoint(Point(5, 6)));
	svgTestRunner.runOnData("intersecting", line, Circle(Point(10, 1), 10), IntersectionData().addIntersectionPoint(Point(0, 1)).addIntersectionPoint((Point(10, 11))));
}

static void line_intersectCircle_test(Line line, Circle circle, IntersectionData expectedOutput)
{
	assert(line.intersect(circle) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(line_intersectCircle_test, line_intersectCircle_test_data, Line, Circle, IntersectionData)

// --- line_intersectLine_test ---
static void line_intersectLine_test_data(const TestWithDataRunner<Line, Line, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Line, Line, IntersectionData> svgTestRunner(
		"line_intersectLine_test", testRunner,
		-6, -6, 30, 30);

	Line line(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("parallel", line, Line(Point(6, 2), Point(12, 8)), IntersectionData());
	svgTestRunner.runOnData("overlapping", line, Line(Point(3, 4), Point(9, 10)), IntersectionData().setInfiniteIntersectionPoints());
	svgTestRunner.runOnData("intersecting", line, Line(Point(12, 1), Point(6, 7)), IntersectionData().addIntersectionPoint(Point(6, 7)));
}

static void line_intersectLine_test(Line sut, Line arg, IntersectionData expectedOutput)
{
	assert(sut.intersect(arg) == expectedOutput);

	// this should give the same result
	assert(arg.intersect(sut) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(line_intersectLine_test, line_intersectLine_test_data, Line, Line, IntersectionData)

// --- segment_intersectLine_test ---
static void segment_intersectLine_test_data(const TestWithDataRunner<Segment, Line, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Segment, Line, IntersectionData> svgTestRunner(
		"segment_intersectLine_test", testRunner,
		-6, -6, 30, 30);

	Segment segm(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("parallel", segm, Line(Point(6, 2), Point(12, 8)), IntersectionData());
	svgTestRunner.runOnData("overlapping", segm, Line(Point(3, 4), Point(9, 10)), IntersectionData().setInfiniteIntersectionPoints());
	svgTestRunner.runOnData("intersecting1", segm, Line(Point(12, 1), Point(6, 7)), IntersectionData().addIntersectionPoint(Point(6, 7)));
	svgTestRunner.runOnData("intersecting2", segm, Line(Point(12, 5), Point(6, 11)), IntersectionData().addIntersectionPoint(Point(8, 9)));
	svgTestRunner.runOnData("intersecting3", segm, Line(Point(6, -1), Point(0, 5)), IntersectionData().addIntersectionPoint(Point(2, 3)));
	svgTestRunner.runOnData("non_intersecting1", segm, Line(Point(15, 5), Point(9, 11)), IntersectionData());
	svgTestRunner.runOnData("non_intersecting2", segm, Line(Point(8, 0), Point(-3, 3)), IntersectionData());
}

static void segment_intersectLine_test(Segment segm, Line line, IntersectionData expectedOutput)
{
	assert(segm.intersect(line) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(segment_intersectLine_test, segment_intersectLine_test_data, Segment, Line, IntersectionData)

// --- segment_intersectSegment_test ---
static void segment_intersectSegment_test_data(const TestWithDataRunner<Segment, Segment, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Segment, Segment, IntersectionData> svgTestRunner(
		"segment_intersectSegment_test", testRunner,
		-6, -6, 30, 30);

	Segment segm(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("parallel", segm, Segment(Point(6, 2), Point(12, 8)), IntersectionData());
	svgTestRunner.runOnData("overlapping", segm, Segment(Point(3, 4), Point(9, 10)), IntersectionData().setInfiniteIntersectionPoints());
	svgTestRunner.runOnData("non_overlapping", segm, Segment(Point(13, 14), Point(19, 20)), IntersectionData());
	svgTestRunner.runOnData("intersecting1", segm, Segment(Point(9, 2), Point(3, 12)), IntersectionData().addIntersectionPoint(Point(6, 7)));
	svgTestRunner.runOnData("intersecting2", segm, Segment(Point(12, 5), Point(6, 11)), IntersectionData().addIntersectionPoint(Point(8, 9)));
	svgTestRunner.runOnData("intersecting3", segm, Segment(Point(6, -1), Point(0, 5)), IntersectionData().addIntersectionPoint(Point(2, 3)));
	svgTestRunner.runOnData("non_intersecting1", segm, Segment(Point(15, 5), Point(9, 11)), IntersectionData());
	svgTestRunner.runOnData("non_intersecting2", segm, Segment(Point(8, 0), Point(-3, 3)), IntersectionData());
	svgTestRunner.runOnData("adjacent1", segm, Segment(Point(8, 9), Point(13, 14)), IntersectionData().addIntersectionPoint(Point(8, 9)));
	svgTestRunner.runOnData("adjacent2", segm, Segment(Point(13, 14), Point(8, 9)), IntersectionData().addIntersectionPoint(Point(8, 9)));
}

static void segment_intersectSegment_test(Segment sut, Segment arg, IntersectionData expectedOutput)
{
	assert(sut.intersect(arg) == expectedOutput);

	// this should give the same result
	assert(arg.intersect(sut) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(segment_intersectSegment_test, segment_intersectSegment_test_data, Segment, Segment, IntersectionData)

// --- segment_containsProjectedPoint_test ---
static void segment_containsProjectedPoint_test_data(const TestWithDataRunner<Segment, Point, bool> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Segment, Point, bool> svgTestRunner(
		"segment_containsProjectedPoint_test", testRunner,
		-6, -6, 30, 30);

	Segment segm(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("out1", segm, Point(-1, 4), false);
	svgTestRunner.runOnData("estr1", segm, Point(1, 4), true);
	svgTestRunner.runOnData("in1", segm, Point(3, 4), true);
	svgTestRunner.runOnData("in2", segm, Point(9, 6), true);
	svgTestRunner.runOnData("estr2", segm, Point(8, 9), true);
	svgTestRunner.runOnData("out2", segm, Point(16, 4), false);
}

static void segment_containsProjectedPoint_test(Segment segm, Point point, bool expectedOutput)
{
	assert(segm.containsProjectedPoint(point) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(segment_containsProjectedPoint_test, segment_containsProjectedPoint_test_data, Segment, Point, bool)

// --- segment_intersectCircleAndReturnNearestPoint_test ---
static void segment_intersectCircleAndReturnNearestPoint_test_data(const TestWithDataRunner<Segment, Circle, bool, Point> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Segment, Circle, bool, Point> svgTestRunner(
		"segment_intersectCircleAndReturnNearestPoint_test", testRunner,
		-6, -6, 30, 30);

	Segment segm(Point(2, 3), Point(8, 9));
	svgTestRunner.runOnData("non_intersecting1", segm, Circle(Point(10, 1), 4), false, Point(0,0));
	svgTestRunner.runOnData("non_intersecting2", segm, Circle(Point(20, 15), 10), false, Point(0,0));
	svgTestRunner.runOnData("non_intersecting3", segm, Circle(Point(-3, -8), 10), false, Point(0,0));
	svgTestRunner.runOnData("containing", segm, Circle(Point(10, 1), 10), false, Point(0,0));
	svgTestRunner.runOnData("tangent", segm, Circle(Point(10, 1), 10 / sqrt(2)), true, Point(5, 6));
	svgTestRunner.runOnData("intersecting1", segm, Circle(Point(20, 11), 15), true, Point(5.645856533065147, 6.64585633065147));
	svgTestRunner.runOnData("intersecting2", segm, Circle(Point(5, 6), sqrt(8)), true, Point(3, 4));
	svgTestRunner.runOnData("intersecting2rev", Segment(segm.pointB(), segm.pointA()), Circle(Point(5, 6), sqrt(8)), true, Point(7, 8));
	svgTestRunner.runOnData("intersecting3", segm, Circle(Point(0, 1), sqrt(8)), true, Point(2, 3));
	svgTestRunner.runOnData("intersecting4", segm, Circle(Point(10, 11), sqrt(8)), true, Point(8, 9));
}

static void segment_intersectCircleAndReturnNearestPoint_test(Segment segm, Circle circle, bool expectedResult, Point expectedContact)
{
	Point contact;
	assert(segm.intersectAndReturnNearestPoint(circle, &contact) == expectedResult);

	if (expectedResult == true)
		assert(contact == expectedContact);
}

REGISTER_TEST_WITH_DATA(segment_intersectCircleAndReturnNearestPoint_test, segment_intersectCircleAndReturnNearestPoint_test_data, Segment, Circle, bool, Point)

// --- polygon_intersectCircle_test ---
static void polygon_intersectCircle_test_data(const TestWithDataRunner<Polygon, Circle, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Polygon, Circle, IntersectionData> svgTestRunner(
		"polygon_intersectCircle_test", testRunner,
		-6, -6, 30, 30);

	const Point verts[] = { Point(1, 1), Point(9, 1), Point(1, 9) };
	Polygon poly(verts, 3);
	svgTestRunner.runOnData("intersecting1", poly, Circle(Point(5, 5), 4),
		IntersectionData()
			.addIntersectionPoint(Point(5, 1))
			.addIntersectionPoint(Point(1, 5))
			.addIntersectionPoint(Point(5-4/sqrt(2), 5+4/sqrt(2)))
			.addIntersectionPoint(Point(5+4/sqrt(2), 5-4/sqrt(2))));
	svgTestRunner.runOnData("intersecting2", poly, Circle(Point(5, 5), 4*sqrt(2)),
		IntersectionData().addIntersectionPoint(Point(9, 1)).addIntersectionPoint(Point(1, 9))
			.addIntersectionPoint(Point(1, 1)));
	svgTestRunner.runOnData("intersecting3", poly, Circle(Point(10, 1), 1),
		IntersectionData().addIntersectionPoint(Point(9, 1)));
	svgTestRunner.runOnData("tangent", poly, Circle(Point(9, 9), 4*sqrt(2)),
		IntersectionData().addIntersectionPoint(Point(5, 5)));
	svgTestRunner.runOnData("non_intersecting", poly, Circle(Point(9, 9), 4),
		IntersectionData());
	svgTestRunner.runOnData("inside", poly, Circle(Point(5, 5), 7),
		IntersectionData().setAreasIntersectFlag());
	svgTestRunner.runOnData("outside", poly, Circle(Point(3, 3), 1),
		IntersectionData().setAreasIntersectFlag());
}

static void polygon_intersectCircle_test(Polygon poly, Circle circle, IntersectionData expectedOutput)
{
	assert(poly.intersect(circle) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(polygon_intersectCircle_test, polygon_intersectCircle_test_data, Polygon, Circle, IntersectionData)

// --- polygon_intersectLine_test ---
static void polygon_intersectLine_test_data(const TestWithDataRunner<Polygon, Line, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Polygon, Line, IntersectionData> svgTestRunner(
		"polygon_intersectLine_test", testRunner,
		-6, -6, 30, 30);

	const Point verts[] = { Point(1, 1), Point(9, 1), Point(1, 9) };
	Polygon poly(verts, 3);
	svgTestRunner.runOnData("intersecting1", poly, Line(Point(3, 3), Point(15, 15)),
		IntersectionData().addIntersectionPoint(Point(1, 1)).addIntersectionPoint(Point(5, 5)));
	svgTestRunner.runOnData("intersecting2", poly, Line(Point(3, 11), Point(15, 23)),
		IntersectionData().addIntersectionPoint(Point(1, 9)));
	svgTestRunner.runOnData("intersecting3", poly, Line(Point(5, -3), Point(5, 10)),
		IntersectionData().addIntersectionPoint(Point(5, 1)).addIntersectionPoint(Point(5, 5)));
	svgTestRunner.runOnData("non_intersecting1", poly, Line(Point(0, 11), Point(12, 23)),
		IntersectionData());
	svgTestRunner.runOnData("non_intersecting2", poly, Line(Point(2, 10), Point(10, 2)),
		IntersectionData());
	svgTestRunner.runOnData("on_border", poly, Line(Point(-1, 11), Point(11, -1)),
		IntersectionData().setInfiniteIntersectionPoints());
}

static void polygon_intersectLine_test(Polygon poly, Line line, IntersectionData expectedOutput)
{
	assert(poly.intersect(line) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(polygon_intersectLine_test, polygon_intersectLine_test_data, Polygon, Line, IntersectionData)

// --- polygon_intersectSegment_test ---
static void polygon_intersectSegment_test_data(const TestWithDataRunner<Polygon, Segment, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Polygon, Segment, IntersectionData> svgTestRunner(
		"polygon_intersectSegment_test", testRunner,
		-6, -6, 30, 30);

	const Point verts[] = { Point(1, 1), Point(9, 1), Point(1, 9) };
	Polygon poly(verts, 3);
	svgTestRunner.runOnData("intersecting1", poly, Segment(Point(3, 3), Point(15, 15)),
		IntersectionData().addIntersectionPoint(Point(5, 5)));
	svgTestRunner.runOnData("intersecting2", poly, Segment(Point(-1, 7), Point(11, 19)),
		IntersectionData().addIntersectionPoint(Point(1, 9)));
	svgTestRunner.runOnData("intersecting3", poly, Segment(Point(5, -3), Point(5, 10)),
		IntersectionData().addIntersectionPoint(Point(5, 1)).addIntersectionPoint(Point(5, 5)));
	svgTestRunner.runOnData("non_intersecting1", poly, Segment(Point(0, 11), Point(12, 23)),
		IntersectionData());
	svgTestRunner.runOnData("non_intersecting2", poly, Segment(Point(2, 10), Point(10, 2)),
		IntersectionData());
	svgTestRunner.runOnData("non_intersecting3", poly, Segment(Point(7, 7), Point(19, 19)),
		IntersectionData());
	svgTestRunner.runOnData("non_intersecting4", poly, Segment(Point(3, 11), Point(15, 23)),
		IntersectionData());
	svgTestRunner.runOnData("on_border", poly, Segment(Point(-1, 11), Point(11, -1)),
		IntersectionData().setInfiniteIntersectionPoints());
}

static void polygon_intersectSegment_test(Polygon poly, Segment segm, IntersectionData expectedOutput)
{
	assert(poly.intersect(segm) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(polygon_intersectSegment_test, polygon_intersectSegment_test_data, Polygon, Segment, IntersectionData)

// --- polygon_intersectPolygon_test ---
static void polygon_intersectPolygon_test_data(const TestWithDataRunner<Polygon, Polygon, IntersectionData> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Polygon, Polygon, IntersectionData> svgTestRunner(
		"polygon_intersectPolygon_test", testRunner,
		-6, -6, 30, 30);

	const Point poly_verts[] = { Point(1, 1), Point(9, 1), Point(1, 9) };
	Polygon poly(poly_verts, 3);

	svgTestRunner.runOnData("self", poly, poly,
		IntersectionData().setInfiniteIntersectionPoints());

	const Point intersecting1_verts[] = { Point(3, 3), Point(11, 3), Point(3, 11) };
	svgTestRunner.runOnData("intersecting1", poly, Polygon(intersecting1_verts, 3),
		IntersectionData().addIntersectionPoint(Point(7, 3)).addIntersectionPoint(Point(3, 7)));

	const Point intersecting2_verts[] = { Point(5, 5), Point(13, 5), Point(5, 13) };
	svgTestRunner.runOnData("intersecting2", poly, Polygon(intersecting2_verts, 3),
		IntersectionData().addIntersectionPoint(Point(5, 5)));

	const Point intersecting3_verts[] = { Point(7, 7), Point(7, -1), Point(-1, 7) };
	svgTestRunner.runOnData("intersecting3", poly, Polygon(intersecting3_verts, 3),
		IntersectionData()
			.addIntersectionPoint(Point(1, 7))
			.addIntersectionPoint(Point(7, 1))
			.addIntersectionPoint(Point(1, 5))
			.addIntersectionPoint(Point(5, 1))
			.addIntersectionPoint(Point(3, 7))
			.addIntersectionPoint(Point(7, 3)));

	const Point intersecting4_verts[] = { Point(7, 7), Point(7, -1), Point(1, 1), Point(-1, 7) };
	svgTestRunner.runOnData("intersecting4", poly, Polygon(intersecting4_verts, 4),
		IntersectionData()
			.addIntersectionPoint(Point(1, 7))
			.addIntersectionPoint(Point(7, 1))
			.addIntersectionPoint(Point(1, 1))
			.addIntersectionPoint(Point(3, 7))
			.addIntersectionPoint(Point(7, 3)));

	const Point intersecting5_verts[] = { Point(9, 1), Point(17, 1), Point(9, 9) };
	svgTestRunner.runOnData("intersecting5", poly, Polygon(intersecting5_verts, 3),
		IntersectionData().addIntersectionPoint(Point(9, 1)));

	const Point sharing_side1_verts[] = { Point(9, 9), Point(9, 1), Point(1, 9) };
	svgTestRunner.runOnData("sharing_side1", poly, Polygon(sharing_side1_verts, 3),
		IntersectionData().setInfiniteIntersectionPoints());

	const Point sharing_side2_verts[] = { Point(9, 9), Point(6, 4), Point(4, 6) };
	svgTestRunner.runOnData("sharing_side2", poly, Polygon(sharing_side2_verts, 3),
		IntersectionData().setInfiniteIntersectionPoints());

	const Point non_intersecting1_verts[] = { Point(7, 7), Point(15, 7), Point(7, 15) };
	svgTestRunner.runOnData("non_intersecting1", poly, Polygon(non_intersecting1_verts, 3),
		IntersectionData());

	const Point non_intersecting2_verts[] = { Point(10, 10), Point(10, 2), Point(2, 10) };
	svgTestRunner.runOnData("non_intersecting2", poly, Polygon(non_intersecting2_verts, 3),
		IntersectionData());

	const Point inside_verts[] = { Point(0, 0), Point(10, 0), Point(12, 15), Point(0, 10) };
	svgTestRunner.runOnData("inside", poly, Polygon(inside_verts, 4),
		IntersectionData().setAreasIntersectFlag());

	const Point outside_verts[] = { Point(2, 2), Point(2, 5), Point(5, 3), Point(5, 2) };
	svgTestRunner.runOnData("outside", poly, Polygon(outside_verts, 4),
		IntersectionData().setAreasIntersectFlag());
}

static void polygon_intersectPolygon_test(Polygon sut, Polygon arg, IntersectionData expectedOutput)
{
	assert(sut.intersect(arg) == expectedOutput);

	// this should give the same result
	assert(arg.intersect(sut) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(polygon_intersectPolygon_test, polygon_intersectPolygon_test_data, Polygon, Polygon, IntersectionData)

// --- polygon_containsPoint_test ---
static void polygon_containsPoint_test_data(const TestWithDataRunner<Polygon, Point, bool> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Polygon, Point, bool> svgTestRunner(
		"polygon_containsPoint_test", testRunner,
		-6, -6, 30, 30);

	const Point a_verts[] = { Point(1, 1), Point(4, 7), Point(7, 1) };
	Polygon a_poly(a_verts, 3);
	svgTestRunner.runOnData("a_inside", a_poly, Point(4, 4), true);
	svgTestRunner.runOnData("a_on_border", a_poly, Point(6, 1), true);
	svgTestRunner.runOnData("a_on_vertex", a_poly, Point(4, 7), true);
	svgTestRunner.runOnData("a_outside1", a_poly, Point(6, 7), false);
	svgTestRunner.runOnData("a_outside2", a_poly, Point(10, 1), false);

	const Point b_verts[] = { Point(4, 7), Point(7, 1), Point(1, 1) };
	Polygon b_poly(b_verts, 3);
	svgTestRunner.runOnData("b_inside", b_poly, Point(4, 4), true);
	svgTestRunner.runOnData("b_on_border", b_poly, Point(6, 1), true);
	svgTestRunner.runOnData("b_on_vertex", b_poly, Point(4, 7), true);
	svgTestRunner.runOnData("b_outside1", b_poly, Point(6, 7), false);
	svgTestRunner.runOnData("b_outside2", b_poly, Point(10, 1), false);

	const Point c_verts[] = { Point(7, 1), Point(1, 1), Point(4, 7) };
	Polygon c_poly(c_verts, 3);
	svgTestRunner.runOnData("c_inside", c_poly, Point(4, 4), true);
	svgTestRunner.runOnData("c_on_border", c_poly, Point(6, 1), true);
	svgTestRunner.runOnData("c_on_vertex", c_poly, Point(4, 7), true);
	svgTestRunner.runOnData("c_outside1", c_poly, Point(6, 7), false);
	svgTestRunner.runOnData("c_outside2", c_poly, Point(10, 1), false);

	const Point d_verts[] = { Point(7, 1), Point(4, 7), Point(1, 1) };
	Polygon d_poly(d_verts, 3);
	svgTestRunner.runOnData("d_inside", d_poly, Point(4, 4), true);
	svgTestRunner.runOnData("d_on_border", d_poly, Point(6, 1), true);
	svgTestRunner.runOnData("d_on_vertex", d_poly, Point(4, 7), true);
	svgTestRunner.runOnData("d_outside1", d_poly, Point(6, 7), false);
	svgTestRunner.runOnData("d_outside2", d_poly, Point(10, 1), false);

	const Point e_verts[] = { Point(1, 1), Point(7, 1), Point(4, 7) };
	Polygon e_poly(e_verts, 3);
	svgTestRunner.runOnData("e_inside", e_poly, Point(4, 4), true);
	svgTestRunner.runOnData("e_on_border", e_poly, Point(6, 1), true);
	svgTestRunner.runOnData("e_on_vertex", e_poly, Point(4, 7), true);
	svgTestRunner.runOnData("e_outside1", e_poly, Point(6, 7), false);
	svgTestRunner.runOnData("e_outside2", e_poly, Point(10, 1), false);

	const Point f_verts[] = { Point(4, 7), Point(1, 1), Point(7, 1) };
	Polygon f_poly(f_verts, 3);
	svgTestRunner.runOnData("f_inside", f_poly, Point(4, 4), true);
	svgTestRunner.runOnData("f_on_border", f_poly, Point(6, 1), true);
	svgTestRunner.runOnData("f_on_vertex", f_poly, Point(4, 7), true);
	svgTestRunner.runOnData("f_outside1", f_poly, Point(6, 7), false);
	svgTestRunner.runOnData("f_outside2", f_poly, Point(10, 1), false);
}

static void polygon_containsPoint_test(Polygon polygon, Point point, bool expectedOutput)
{
	assert(polygon.containsPoint(point) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(polygon_containsPoint_test, polygon_containsPoint_test_data, Polygon, Point, bool)

// --- capsule_intersect_test ---
static void capsule_intersect_test_data(const TestWithDataRunner<Capsule, Capsule, bool> &testRunner)
{
	TestWithDataRunnerSvgDecorator<Capsule, Capsule, bool> svgTestRunner(
		"capsule_intersect_test", testRunner,
		-6, -6, 40, 40);

	Capsule caps(Point(3, 3), Point(24, 24), 3);
	svgTestRunner.runOnData("self", caps, caps, true);
	svgTestRunner.runOnData("intersecting1", caps, Capsule(Point(3, 15), Point(24, 12), 3), true);
	svgTestRunner.runOnData("intersecting2", caps, Capsule(Point(-1, 3), Point(-2, 3), 3), true);
	svgTestRunner.runOnData("intersecting3", caps, Capsule(Point(12, 18), Point(0, 30), 3), true);
	svgTestRunner.runOnData("intersecting4", caps, Capsule(Point(3, 3), Point(15, 15), 5), true);
	svgTestRunner.runOnData("inside", caps, Capsule(Point(3, 3), Point(24, 24), 1), true);
	svgTestRunner.runOnData("outside", caps, Capsule(Point(2, 3), Point(25, 24), 5), true);
	svgTestRunner.runOnData("non_intersecting", caps, Capsule(Point(3, 15), Point(16, 28), 3), false);
}

static void capsule_intersect_test(Capsule sut, Capsule arg, bool expectedOutput)
{
	assert(sut.intersect(arg) == expectedOutput);

	// this should give the same result
	assert(arg.intersect(sut) == expectedOutput);
}

REGISTER_TEST_WITH_DATA(capsule_intersect_test, capsule_intersect_test_data, Capsule, Capsule, bool)
