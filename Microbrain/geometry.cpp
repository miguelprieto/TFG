#include "geometry.h"

#include <stdlib.h>
#include <stdio.h>

// Nota: alcune funzioni di questo file provengono dalla libreria gbmath
// https://code.google.com/p/gbmath/source/browse/#svn%2Ftrunk

double normalize_angle(double a)
{
	if (a > PI)
		a = a - TWO_PI;
	if (a < -PI)
		a = a + TWO_PI;
	return a;
}

double normalize_angle_degrees(double a)
{
	if (a > 180)
		a = a - 360;
	if (a < -180)
		a = a + 360;
	return a;
}

static double sqr(double v)
{
	return v * v;
}

static bool fuzzyCompare(double a, double b)
{
	return fabs(a - b) < .00001;
}

double distance2(const Point &a, const Point &b)
{
	const double dx = a.x - b.x, dy = a.y - b.y;
	return sqr(dx) + sqr(dy);
}

double distance(const Point &a, const Point &b)
{
	return sqrt(distance2(a, b));
}

Point localToGlobal(const Point &robot_pos, const double theta, const Point &offset)
{
	double cos_t = cos(theta);
	double sin_t = sin(theta);
	double offset_x_global = robot_pos.x + offset.x * cos_t - offset.y * sin_t;
	double offset_y_global = robot_pos.y + offset.x * sin_t + offset.y * cos_t;
	return Point(offset_x_global, offset_y_global);
}

// --- Point ---
Point::Point(double x, double y)
: x(x), y(y)
{
}

bool Point::operator==(const Point &other) const
{
	return fuzzyCompare(x, other.x) && fuzzyCompare(y, other.y);
}

bool Point::operator!=(const Point &other) const
{
	return !operator==(other);
}

// --- IntersectionData ---
IntersectionData::IntersectionData()
: m_areasIntersect(false), m_numPoints(0)
{
}

IntersectionData &IntersectionData::setAreasIntersectFlag()
{
	m_areasIntersect = true;
	return *this;
}

IntersectionData &IntersectionData::setInfiniteIntersectionPoints()
{
	setAreasIntersectFlag();
	m_numPoints = UINT_MAX;
	return *this;
}

IntersectionData &IntersectionData::addIntersectionPoint(const Point &p)
{
	setAreasIntersectFlag();

	if (m_numPoints < MAX_INTERSECTIONS)
		m_points[m_numPoints++] = p;
	else
		printf("Errore! IntersectionData::addIntersectionPoint richiamato con m_numPoints=%u (MAX_INTERSECTIONS=%u)\n", m_numPoints, MAX_INTERSECTIONS);

	return *this;
}

IntersectionData &IntersectionData::merge(const IntersectionData &other)
{
	if (other.m_areasIntersect)
		m_areasIntersect = true;

	if (m_numPoints != UINT_MAX)
	{
		if (other.m_numPoints == UINT_MAX)
		{
			setInfiniteIntersectionPoints();
		}
		else
		{
			for (unsigned int i = 0; i < other.m_numPoints; i++)
			{
				// Aggiungi punti solo se non sono già presenti
				bool found = false;
				for (unsigned int j = 0; j < m_numPoints && !found; j++)
				{
					if (other.m_points[i] == m_points[j])
						found = true;
				}

				if (!found)
					m_points[m_numPoints++] = other.m_points[i];
			}
		}
	}

	return *this;
}

bool IntersectionData::areasIntersect() const
{
	return m_areasIntersect;
}

unsigned int IntersectionData::intersectionCount() const
{
	return m_numPoints;
}

const Point &IntersectionData::intersectionPoint(unsigned int i) const
{
	return m_points[i];
}

bool IntersectionData::operator==(const IntersectionData &other) const
{
	if (m_areasIntersect != other.m_areasIntersect)
		return false;
	if (m_numPoints != other.m_numPoints)
		return false;

	for (unsigned int i = 0; i < m_numPoints && m_numPoints != UINT_MAX; i++)
	{
		bool foundThisInOther = false;
		bool foundOtherInThis = false;

		// Controlla che il punto m_points[i] sia contenuto in other.m_points
		// e che other.m_points[i] sia contenuto in m_points
		for (unsigned int j = 0; j < m_numPoints; j++)
		{
			if (m_points[i] == other.m_points[j])
				foundThisInOther = true;
			if (other.m_points[i] == m_points[j])
				foundOtherInThis = true;
		}

		if (!foundThisInOther || !foundOtherInThis)
			return false;
	}

	return true;
}

bool IntersectionData::operator!=(const IntersectionData &other) const
{
	return !operator==(other);
}

// --- Circle ---
Circle::Circle(const Point &center, double radius)
: m_center(center), m_radius(radius)
{
}

const Point &Circle::center() const
{
	return m_center;
}

double Circle::radius() const
{
	return m_radius;
}

IntersectionData Circle::intersect(const Circle &c2) const
{
	const double dc = distance(m_center, c2.m_center);
	const double dr = fabs(m_radius - c2.m_radius);

	if (fuzzyCompare(dc, 0))
	{
		// Cerchi coincidenti o semplicemente concentrici?
		if (fuzzyCompare(dr, 0))
			return IntersectionData().setInfiniteIntersectionPoints();
		else
			return IntersectionData().setAreasIntersectFlag();
	}
	else if (dc < dr) // Un cerchio è interamente contenuto dentro l'altro
	{
		return IntersectionData().setAreasIntersectFlag();
	}

	const double X1 = m_center.x;
	const double Y1 = m_center.y;
	const double R1 = m_radius;
	const double X2 = c2.m_center.x;
	const double Y2 = c2.m_center.y;
	const double R2 = c2.m_radius;

	const double vx = X2 - X1;
	const double vy = Y2 - Y1;

	if (fuzzyCompare(dc, R1 + R2)) // Cerchi tangenti esternamente
	{
		const double dir = atan2(vy, vx);
		const Point pContact(X1 + cos(dir) * R1, Y1 + sin(dir) * R1);
		return IntersectionData().addIntersectionPoint(pContact);
	}

	const double vv = sqr(vx) + sqr(vy);
	if (sqr(R1 + R2) < vv)
		return IntersectionData(); // nessuna intersezione

	const double a = (R1 + R2) * (R1 - R2) / 2 / vv + 0.5f;
	const double b = sqr(R1) / vv - sqr(a);
	const double c = sqrt(b > 0 ? b : 0);

	Point pContact1;
	pContact1.x = X1 + a*vx - c*vy;
	pContact1.y = Y1 + a*vy + c*vx;

	if (fuzzyCompare(dc, dr)) // Cerchi tangenti internamente
		return IntersectionData().addIntersectionPoint(pContact1);

	Point pContact2;
	pContact2.x = X1 + a*vx + c*vy;
	pContact2.y = Y1 + a*vy - c*vx;

	return IntersectionData().addIntersectionPoint(pContact1).addIntersectionPoint(pContact2);
}

bool Circle::containsPoint(const Point &point) const
{
	return (distance2(m_center, point) <= sqr(m_radius));
}

// --- Line ---
Line::Line(const Point &pointA, const Point &pointB)
: m_pointA(pointA), m_pointB(pointB)
{
}

const Point &Line::pointA() const
{
	return m_pointA;
}

const Point &Line::pointB() const
{
	return m_pointB;
}

Point Line::projectPoint(const Point &pnt) const
{
	const double b1 = pnt.x * (m_pointA.x - m_pointB.x) + pnt.y * (m_pointA.y - m_pointB.y);
	const double b2 =  m_pointA.x * m_pointB.y - m_pointA.y * m_pointB.x;

	double pt_to_x, pt_to_y = distance2(m_pointA, m_pointB);
	double det_k = b1 * (m_pointA.x - m_pointB.x) - b2 * (m_pointA.y - m_pointB.y);

	pt_to_x = det_k / pt_to_y;
	det_k = (m_pointA.x -  m_pointB.x) * b2 + (m_pointA.y -  m_pointB.y) * b1;
	pt_to_y = det_k / pt_to_y;

	return Point(pt_to_x, pt_to_y);
}

double Line::pointDistance(const Point &pnt) const
{
	return distance(pnt, projectPoint(pnt));
}

IntersectionData Line::intersect(const Circle &cir) const
{
	IntersectionData result;
	double s0, s1;

	if (calcCircleIntersectionDistances(cir, &s0, &s1))
	{

		const double norm_f = 1.0f / distance(m_pointA, m_pointB);
		const double dirx = (m_pointB.x - m_pointA.x) * norm_f;
		const double diry = (m_pointB.y - m_pointA.y) * norm_f;

		result.addIntersectionPoint(Point(m_pointA.x + dirx*s0, m_pointA.y + diry*s0));

		if (!fuzzyCompare(s0, s1))
			result.addIntersectionPoint(Point(m_pointA.x + dirx*s1, m_pointA.y + diry*s1));
	}

	return result;
}

IntersectionData Line::intersect(const Line &l) const
{
	const double a_x1 = m_pointA.x;
	const double a_y1 = m_pointA.y;
	const double a_x2 = m_pointB.x;
	const double a_y2 = m_pointB.y;
	const double b_x1 = l.m_pointA.x;
	const double b_y1 = l.m_pointA.y;
	const double b_x2 = l.m_pointB.x;
	const double b_y2 = l.m_pointB.y;

	const double det = a_x1 * (b_y2 - b_y1) + a_x2 * (b_y1 - b_y2) + a_y2 * (b_x2 - b_x1) + a_y1 * (b_x1 - b_x2);
	IntersectionData result;

	if (det != 0)
	{
		Point pContact;
		pContact.x = (a_x1 * (b_x1 * b_y2 - b_x2 * b_y1) + a_x2 * (b_x2 * b_y1 - b_x1 * b_y2) + a_x1 * a_y2 * (b_x2 - b_x1) + a_x2 * a_y1 * (b_x1 - b_x2)) / det;
		pContact.y = (a_y2 * (a_x1 * (b_y2 - b_y1) - b_x1 * b_y2 + b_x2 * b_y1) + a_y1 * (b_x1 * b_y2 + a_x2 * (b_y1 - b_y2) - b_x2 * b_y1)) / det;
		result.addIntersectionPoint(pContact);
	}
	else if (projectPoint(l.m_pointA) == l.m_pointA) // rette coincidenti
	{
		result.setInfiniteIntersectionPoints();
	}

	return result;
}

bool Line::calcCircleIntersectionDistances(const Circle &cir, double *ps0, double *ps1) const
{
	const double vx = m_pointA.x - cir.center().x;
	const double vy = m_pointA.y - cir.center().y;

	const double norm_f = 1.0f / distance(m_pointA, m_pointB);
	const double dirx = (m_pointB.x - m_pointA.x) * norm_f;
	const double diry = (m_pointB.y - m_pointA.y) * norm_f;

	const double b = 2.0f * (dirx*vx + diry*vy);
	const double c = sqr(vx) + sqr(vy) - sqr(cir.radius());
	double discriminant = sqr(b) - (4.0f * c);

	if (discriminant < 0.0f)
	{
		// workaround per discriminant = -0.0
		if (fuzzyCompare(discriminant, 0))
			discriminant = 0;
		else
			return false;
	}

	discriminant = sqrt(discriminant);

	double s0 = (-b + discriminant) / 2.0f;
	double s1 = (-b - discriminant) / 2.0f;

	if (ps0) *ps0 = s0;
	if (ps1) *ps1 = s1;

	return true;
}

// --- Segment ---
Segment::Segment(const Point &pointA, const Point &pointB)
: m_pointA(pointA), m_pointB(pointB)
{
}

const Point &Segment::pointA() const
{
	return m_pointA;
}

const Point &Segment::pointB() const
{
	return m_pointB;
}

IntersectionData Segment::intersect(const Circle &c) const
{
	IntersectionData result;
	double s0, s1;

	if (Line(m_pointA, m_pointB).calcCircleIntersectionDistances(c, &s0, &s1))
	{
		const double len = distance(m_pointA, m_pointB);
		const double norm_f = 1.0f / len;
		const double dirx = (m_pointB.x - m_pointA.x) * norm_f;
		const double diry = (m_pointB.y - m_pointA.y) * norm_f;

		if (s0 >= 0.0f && s0 <= len)
			result.addIntersectionPoint(Point(m_pointA.x + dirx*s0, m_pointA.y + diry*s0));

		if (s1 >= 0.0f && s1 <= len)
			result.addIntersectionPoint(Point(m_pointA.x + dirx*s1, m_pointA.y + diry*s1));
	}

	return result;
}

IntersectionData Segment::intersect(const Line &l) const
{
	IntersectionData inters = Line(m_pointA, m_pointB).intersect(l);

	switch (inters.intersectionCount())
	{
		case 0:
			return IntersectionData(); // rette parallele
		case 1:
			// La retta considerata e la retta che include il
			// segmento segmento hanno un punto in comune. Tale
			// punto appartiene al segmento?
			if (containsProjectedPoint(inters.intersectionPoint(0)))
				return IntersectionData().addIntersectionPoint(inters.intersectionPoint(0));
			else
				return IntersectionData(); // nessuna intersezione
		case UINT_MAX:
			return IntersectionData().setInfiniteIntersectionPoints();
		default:
			abort();
	}
}

IntersectionData Segment::intersect(const Segment &segm) const
{
	IntersectionData inters = intersect(Line(segm.m_pointA, segm.m_pointB));

	switch (inters.intersectionCount())
	{
		case 0:
			return IntersectionData();
		case 1:
			// Il punto trovato appartiene anche a segm?
			if (segm.containsProjectedPoint(inters.intersectionPoint(0)))
				return IntersectionData().addIntersectionPoint(inters.intersectionPoint(0));
			else
				return IntersectionData(); // nessuna intersezione
		case UINT_MAX:
			// I due segmenti giacciono sulla stessa retta, ma
			// hanno davvero punti in comune?

			// Consideriamo il caso in cui abbiano solo un estremo in comune
			if ((m_pointB.x - m_pointA.x) * (segm.m_pointB.x - segm.m_pointA.x) + (m_pointB.y - m_pointA.y) * (segm.m_pointB.y - segm.m_pointA.y) > 0) // verso concorde?
			{
				if (m_pointA == segm.m_pointB)
					return IntersectionData().addIntersectionPoint(m_pointA);
				if (m_pointB == segm.m_pointA)
					return IntersectionData().addIntersectionPoint(m_pointB);
			}
			else
			{
				if (m_pointA == segm.m_pointA)
					return IntersectionData().addIntersectionPoint(m_pointA);
				if (m_pointB == segm.m_pointB)
					return IntersectionData().addIntersectionPoint(m_pointB);
			}

			// Altri caso: hanno una porzione in comune
			if (containsProjectedPoint(segm.m_pointA) || segm.containsProjectedPoint(m_pointA))
				return IntersectionData().setInfiniteIntersectionPoints();
			else
				return IntersectionData(); // nessuna intersezione
		default:
			abort();
	}
}

bool Segment::containsProjectedPoint(const Point &point) const
{
	const double len = distance(m_pointA, m_pointB);
	const double norm_f = 1.0f / len;
	const double dirx = (m_pointB.x - m_pointA.x) * norm_f;
	const double diry = (m_pointB.y - m_pointA.y) * norm_f;
	const double dx = point.x - m_pointA.x;
	const double dy = point.y - m_pointA.y;
	const double s = dirx * dx + diry * dy;

	return ((s > 0.0f || fuzzyCompare(s, 0)) && (s < len || fuzzyCompare(s, len)));
}

bool Segment::intersectAndReturnNearestPoint(const Circle &cir, Point *pContact) const
{
	IntersectionData inters = intersect(cir);
	double d0, d1;

	switch (inters.intersectionCount())
	{
		case 0:
			return false;
		case 1:
			*pContact = inters.intersectionPoint(0);
			return true;
		case 2:
			d0 = distance2(inters.intersectionPoint(0), m_pointA);
			d1 = distance2(inters.intersectionPoint(1), m_pointA);
			*pContact = inters.intersectionPoint((d0 < d1) ? 0 : 1);
			return true;
		default:
			abort();
	}
}

// --- Polygon ---
Polygon::Polygon(const Point vertexArray[], unsigned int vertexCount)
: m_vertexCount(0)
{
	if (vertexCount < 3)
	{
		printf("Errore! Creazione di un poligono degenere (vertexCount=%u)\n", vertexCount);
	}
	else if (vertexCount > MAX_VERTICES)
	{
		printf("Errore! Creazione di un poligono con troppi vertici (vertexCount=%u, MAX_VERTICES=%u)\n", vertexCount, MAX_VERTICES);
	}
	else
	{
		m_vertexCount = vertexCount;
		while (vertexCount-- != 0)
			m_vertexArray[vertexCount] = vertexArray[vertexCount];
	}
}

const Point &Polygon::vertexAtIndex(unsigned int index) const
{
	return m_vertexArray[index];
}

const Point *Polygon::vertexArray() const
{
	return m_vertexArray;
}

unsigned int Polygon::vertexCount() const
{
	return m_vertexCount;
}

Segment Polygon::segmentAtIndex(unsigned int index) const
{
	if (index == 0)
		return Segment(m_vertexArray[m_vertexCount - 1], m_vertexArray[0]);
	else
		return Segment(m_vertexArray[index - 1], m_vertexArray[index]);
}

IntersectionData Polygon::intersect(const Circle &circle) const
{
	IntersectionData result;

	for (unsigned int i = 0; i < m_vertexCount; i++)
		result.merge(segmentAtIndex(i).intersect(circle));

	if (!result.areasIntersect() && circle.containsPoint(m_vertexArray[0]))
		result.setAreasIntersectFlag();

	if (!result.areasIntersect() && containsPoint(circle.center()))
		result.setAreasIntersectFlag();

	return result;
}

IntersectionData Polygon::intersect(const Line &line) const
{
	IntersectionData result;

	for (unsigned int i = 0; i < m_vertexCount; i++)
		result.merge(segmentAtIndex(i).intersect(line));

	return result;
}

IntersectionData Polygon::intersect(const Segment &segm) const
{
	IntersectionData result;

	for (unsigned int i = 0; i < m_vertexCount; i++)
		result.merge(segm.intersect(segmentAtIndex(i)));

	return result;
}

IntersectionData Polygon::intersect(const Polygon &poly) const
{
	IntersectionData result;

	for (unsigned int i = 0; i < m_vertexCount; i++)
		result.merge(poly.intersect(segmentAtIndex(i)));

	if (!result.areasIntersect() && poly.containsPoint(m_vertexArray[0]))
		result.setAreasIntersectFlag();

	if (!result.areasIntersect() && containsPoint(poly.m_vertexArray[0]))
		result.setAreasIntersectFlag();

	return result;
}

bool Polygon::containsPoint(const Point &point) const
{
	// Controlla che i prodotti vettoriali
	//	estremoA-point CROSS estremoA-estremoB
	// abbiano, per ogni segmento, tutti lo stesso segno

	// Primo segmento considerato: da ultimo a primo vertice
	double v1x = m_vertexArray[m_vertexCount-1].x;
	double v1y = m_vertexArray[m_vertexCount-1].y;
	double v2x = m_vertexArray[0].x;
	double v2y = m_vertexArray[0].y;
	double cross = (point.x - v1x) * (v2y - v1y) - (point.y - v1y) * (v2x - v1x);
	double dist = cross / distance(m_vertexArray[m_vertexCount-1], m_vertexArray[0]);

	// Gestiamo a parte il caso in cui il punto appartenga al segmento
	if (fuzzyCompare(dist, 0) && Segment(m_vertexArray[m_vertexCount-1], m_vertexArray[0]).containsProjectedPoint(point))
		return true;

	// Ripetiamo l'operazione per gli altri segmenti, controllando che il
	// segno dei prodotti vettoriali non cambi
	double old_cross = cross;
	for (unsigned int i = 1; i < m_vertexCount; i++)
	{
		v1x = m_vertexArray[i-1].x;
		v1y = m_vertexArray[i-1].y;
		v2x = m_vertexArray[i].x;
		v2y = m_vertexArray[i].y;
		cross = (point.x - v1x) * (v2y - v1y) - (point.y - v1y) * (v2x - v1x);
		dist = cross / distance(m_vertexArray[i-1], m_vertexArray[i]);

		// Gestiamo a parte il caso in cui il punto appartenga al segmento
		if (fuzzyCompare(dist, 0) && Segment(m_vertexArray[i-1], m_vertexArray[i]).containsProjectedPoint(point))
			return true;

		if (cross * old_cross < 0)
			return false; // è cambiato il segno

		if (!fuzzyCompare(cross, 0))
			old_cross = cross;
	}

	return true;
}

// --- Capsule ---
Capsule::Capsule(const Point &pointA, const Point &pointB, double radius)
: m_estrA(pointA, radius), m_estrB(pointB, radius), m_body(({
		// Calcola vettore ortogonale al segmento AB e di modulo pari a radius
		const double dir = HALF_PI + atan((pointB.y - pointA.y) / (pointB.x - pointA.x));
		const double dx = cos(dir) * radius;
		const double dy = sin(dir) * radius;

		// Calcola coordinate dei vertici del rettangolo interno
		Point verts[4];
		verts[0] = Point(pointA.x + dx, pointA.y + dy);
		verts[1] = Point(pointA.x - dx, pointA.y - dy);
		verts[2] = Point(pointB.x - dx, pointB.y - dy);
		verts[3] = Point(pointB.x + dx, pointB.y + dy);

		// Crea oggetto poligono con i vertici appena calcolati
		Polygon(verts, 4);
	}))
{
}

const Point &Capsule::pointA() const
{
	return m_estrA.center();
}

const Point &Capsule::pointB() const
{
	return m_estrB.center();
}

double Capsule::radius() const
{
	return m_estrA.radius();
}

bool Capsule::intersect(const Capsule &other) const
{
	if (m_estrA.intersect(other.m_estrA).areasIntersect())
		return true;
	else if (m_estrA.intersect(other.m_estrB).areasIntersect())
		return true;
	else if (m_estrB.intersect(other.m_estrA).areasIntersect())
		return true;
	else if (m_estrB.intersect(other.m_estrB).areasIntersect())
		return true;
	else if (other.m_body.intersect(m_estrA).areasIntersect())
		return true;
	else if (other.m_body.intersect(m_estrB).areasIntersect())
		return true;
	else if (m_body.intersect(other.m_estrA).areasIntersect())
		return true;
	else if (m_body.intersect(other.m_estrB).areasIntersect())
		return true;
	else if (m_body.intersect(other.m_body).areasIntersect())
		return true;
	else
		return false;
}
