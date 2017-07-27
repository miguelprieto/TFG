/*
 * geometry.h
 */

#ifndef __GEOMETRY_H
#define __GEOMETRY_H

#include <limits.h>
#include <math.h>

#ifdef __XC16__
#include <dsp.h> // for PI
#else
#define PI M_PI
#endif

#define TWO_PI	(2 * PI)
#define HALF_PI	(PI / 2)

#define TO_DEGREES(a)  ((a) * (180.0/PI))
#define TO_RADIANS(a)  ((a) * (PI/180.0))

struct Point
{
	Point() = default; // crea punto non inizializzato
	Point(double x, double y);

	// fuzzy compare
	bool operator==(const Point &other) const;
	bool operator!=(const Point &other) const;

	double x, y;
};

double normalize_angle(double a);
double normalize_angle_degrees(double a);
double distance(const Point &a, const Point &b);
double distance2(const Point &a, const Point &b);
Point localToGlobal(const Point &robot_pos, const double theta, const Point &offset);

// Struttura contenente informazioni sull'intersezione di due oggetti
class IntersectionData
{
	public:
		// Gli oggetti considerati hanno una porzione di area o perimetro in comune?
		bool areasIntersect() const;

		// Punti di intersezione tra perimetri, curve e/o segmenti degli oggetti considerati
		unsigned int intersectionCount() const; // numero di punti di intersezione (0 se nessuno, UINT_MAX se infiniti)
		const Point &intersectionPoint(unsigned int i) const; // restituisce punto di intersezione i-esimo

		bool operator==(const IntersectionData &other) const;
		bool operator!=(const IntersectionData &other) const;

		// Metodi utilizzati per costruire oggetti di tipo IntersectionData
		// Tutti restituiscono un riferimento a *this
		IntersectionData();
		IntersectionData &setAreasIntersectFlag();
		IntersectionData &setInfiniteIntersectionPoints();
		IntersectionData &addIntersectionPoint(const Point &p);
		IntersectionData &merge(const IntersectionData &other);

		constexpr static unsigned int MAX_INTERSECTIONS = 8;

	private:
		bool m_areasIntersect;
		unsigned int m_numPoints;
		Point m_points[MAX_INTERSECTIONS];
};

// Circonferenza
class Circle
{
	public:
		Circle() = delete;

		// Circonferenza di dato centro e raggio
		Circle(const Point &center, double radius);
		const Point &center() const;
		double radius() const;

		IntersectionData intersect(const Circle &c) const;

		bool containsPoint(const Point &point) const;

	private:
		Point m_center;
		double m_radius;
};

// Retta
class Line
{
	friend class Segment;

	public:
		Line() = delete;

		// Retta passante per due punti
		Line(const Point &pointA, const Point &pointB);
		const Point &pointA() const;
		const Point &pointB() const;

		IntersectionData intersect(const Circle &c) const;
		IntersectionData intersect(const Line &l) const;

		Point projectPoint(const Point &pnt) const;
		double pointDistance(const Point &pnt) const;

	private:
		Point m_pointA, m_pointB;

		bool calcCircleIntersectionDistances(const Circle &cir, double *ps0, double *ps1) const;
};

// Segmento
class Segment
{
	public:
		Segment() = delete;

		// Segmento tra due punti
		Segment(const Point &pointA, const Point &pointB);
		const Point &pointA() const;
		const Point &pointB() const;

		IntersectionData intersect(const Circle &c) const;
		IntersectionData intersect(const Line &l) const;
		IntersectionData intersect(const Segment &s) const;

		bool containsProjectedPoint(const Point &point) const;

		// Calcola l'eventuale punto di intersezione più vicino ad a
		bool intersectAndReturnNearestPoint(const Circle &cir, Point *pContact) const;

	private:
		Point m_pointA, m_pointB;
};

// Poligono convesso
class Polygon
{
	public:
		Polygon() = delete;

		// Poligono convesso delimitato da una spezzata chiusa
		Polygon(const Point vertexArray[], unsigned int vertexCount);
		const Point &vertexAtIndex(unsigned int index) const;
		const Point *vertexArray() const;
		unsigned int vertexCount() const;

		IntersectionData intersect(const Circle &c) const;
		IntersectionData intersect(const Line &l) const;
		IntersectionData intersect(const Segment &s) const;
		IntersectionData intersect(const Polygon &s) const;

		bool containsPoint(const Point &point) const;

		constexpr static unsigned int MAX_VERTICES = 4;

	private:
		Segment segmentAtIndex(unsigned int index) const;

		unsigned int m_vertexCount;
		Point m_vertexArray[MAX_VERTICES];
};

// Capsula
class Capsule
{
	public:
		Capsule() = delete;

		// Luogo dei punti la cui distanza dal segmento AB è minore o uguale a radius
		Capsule(const Point &pointA, const Point &pointB, double radius);
		const Point &pointA() const;
		const Point &pointB() const;
		double radius() const;

		// Interseca due capsule e restituisce true se hanno almeno un
		// punto in comune
		bool intersect(const Capsule &other) const;

	private:
		Circle m_estrA, m_estrB; // cerchi alle estremità
		Polygon m_body; // rettangolo centrale
};

// Differenza tra due angoli, compresa tra -180 e 180
#define ANG_DIFF(ANG_A, ANG_B)	( fmod((ANG_B) - (ANG_A) + 360*2 + 180, 360) - 180 )

#endif
