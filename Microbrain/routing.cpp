/*
 * routing.c
 */

#include "defines.h"

#include <stdio.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <libpic30++.h>

#include "routing.h"
#include "bus_interface.h"
#include "wait.h"
#include "controller_presets.h"

/* Questo file contiene le coordinate e le liste di adiacenza dei nodi del
 * grafo. Viene generato automaticamente in base a routing.graph.
 * Questo file definisce inoltre
 *	#define NODECOUNT <num-di-nodi-presenti-nel-grafo>
 *	static GraphNode *const allNodes[NODECOUNT]; -- array di puntatori a ciascun nodo
 */
#include "graph_data.gen"

// closePointsInDirection chiude punti entro questa distanza. DEVE corrispondere alla distanza di visione dell'ObstacleAvoidance
#define DIST_TRESHOLD 600 // mm

// closePointsInDirection chiude punti a +/- ANG_TRESHOLD gradi, tenere sincronizzato con ObstacleAvoidance/main.c
#define ANG_TRESHOLD 50 // gradi

// Raggio dell'avversario
#define ENEMY_RADIUS	110 // mm

// Puntatore al nodo a partire dal quale è stato eseguito dijkstra, oppure NULL
// se i dati contenuti nei campi dist e prev dei nodi non sono validi
static GraphNode *dijkstra_sp = NULL;

// Vertici dell'ultimo path calcolato da getPath
static GraphNode *path[NODECOUNT] __attribute__((far));

GraphNode::GraphNode(int x, int y, const char *name, AdjNode *adjList, int adjCount)
: pos(x, y), name(name), adjList(adjList), adjCount(adjCount)
{
}

void GraphNode::AdjNode::open()
{
	if (closed_obstacle == true)
	{
		closed_obstacle = false;
		dijkstra_sp = NULL; // i risultati precedentemente calcolati sono diventati obsoleti
	}
}

void GraphNode::AdjNode::close()
{
	ct = game_timer;

	if (closed_obstacle == false)
	{
		closed_obstacle = true;
		dijkstra_sp = NULL; // i risultati precedentemente calcolati sono diventati obsoleti
	}
}

void GraphNode::openAllEdges()
{
	for (int i = 0; i < adjCount; i++)
		adjList[i].open();
}

void GraphNode::openEdgesToBeOpened()
{
	for (int i = 0; i < adjCount; i++)
	{
		if (adjList[i].isToBeOpened())
			adjList[i].open();
	}
}

void GraphNode::closeAllEdges()
{
	for (int i = 0; i < adjCount; i++)
		adjList[i].open();
}

void GraphNode::closeEdgesIntersecting(const Capsule &capsule, double edge_radius, const Point &robot_pos, double obstacle_dir)
{
	for (int i = 0; i < adjCount; i++)
	{
		GraphNode *n = adjList[i].node;

		if (capsule.intersect(Capsule(pos, n->pos, edge_radius)))
		{
			double edge_dir = atan2(n->pos.y - pos.y, n->pos.x - pos.x);

			// La direzione dell'arco è concorde a quella in cui abbiamo visto l'avversario?
			bool in_avanti = fabs(normalize_angle(obstacle_dir - edge_dir)) < HALF_PI;

			// Se l'arco è in_avanti, lo dobbiamo chiudere sempre.
			// Altrimenti, solo se gli estremi si trovano entrambi
			// davanti al robot
			double dotpr = (n->pos.x - robot_pos.x) * cos(obstacle_dir) + (n->pos.y - robot_pos.y) * sin(obstacle_dir);
			if (!in_avanti && dotpr < 0)
			{
				//printf("NOT closing edge between %s [%.0f, %.0f] and %s [%.0f, %.0f]\n",
				//	printableName(), pos.x, pos.y,
				//	n->printableName(), n->pos.x, n->pos.y);
				continue;
			}

			//printf("closed edge between %s [%.0f, %.0f] and %s [%.0f, %.0f]\n",
			//	printableName(), pos.x, pos.y,
			//	n->printableName(), n->pos.x, n->pos.y);

			adjList[i].close();
		}
	}
}

GraphNode::DijkstraMinHeap::DijkstraMinHeap(GraphNode *sp, GraphNode **scratchpad)
{
	heap = scratchpad;
	size = NODECOUNT;

	for (unsigned int i = 0; i < size; i++)
	{
		GraphNode *n = allNodes[i];
		n->dist = (n == sp) ? 0 : DBL_MAX;
		n->prev = NULL;
		n->minHeapIndex = i;
		heap[i] = n;
	}

	swap(0, sp->minHeapIndex);
}

bool GraphNode::DijkstraMinHeap::isEmpty() const
{
	return (size == 0);
}

GraphNode *GraphNode::DijkstraMinHeap::extractMin()
{
	swap(0, --size);
	minHeapify(0);
	return heap[size];
}

void GraphNode::DijkstraMinHeap::minHeapify(unsigned int i)
{
	const unsigned int left = 2*i + 1;
	const unsigned int right = 2*i + 2;
	unsigned int lowest = i;

	if (left < size && isLessThan(left, lowest))
		lowest = left;
	if (right < size && isLessThan(right, lowest))
		lowest = right;

	if (lowest != i)
	{
		swap(lowest, i);
		minHeapify(lowest);
	}
}

void GraphNode::DijkstraMinHeap::minHeapifyR(unsigned int i)
{
	if (i > 0)
	{
		const unsigned int parent = (i-1) / 2;
		if (isLessThan(i, parent))
		{
			swap(parent, i);
			minHeapifyR(parent);
		}
	}
}

bool GraphNode::DijkstraMinHeap::isLessThan(unsigned int i, unsigned int j)
{
	return heap[i]->dist < heap[j]->dist;
}

void GraphNode::DijkstraMinHeap::swap(unsigned int i, unsigned int j)
{
	GraphNode *t = heap[j];
	heap[j] = heap[i];
	heap[i] = t;
	heap[i]->minHeapIndex = i;
	heap[j]->minHeapIndex = j;
}

// Calcola albero dei cammini minimi radicato in sp
// "scratchpad" è lo spazio di memoria usato durante la computazione, e deve
// poter contenere NODECOUNT elementi
void GraphNode::dijkstra(GraphNode *sp, GraphNode **scratchpad)
{
	// Se abbiamo già applicato l'algoritmo di Dijkstra con gli stessi input,
	// non c'è bisogno di ricalcolare nulla!
	if (dijkstra_sp == sp)
		return;
	dijkstra_sp = sp;

	DijkstraMinHeap dijMinHeap(sp, scratchpad);
	GraphNode *actual_point;

	while (!dijMinHeap.isEmpty() && (actual_point = dijMinHeap.extractMin())->dist != DBL_MAX)
	{
		// Esploriamo nodi adiacenti, ignorando i nodi chiusi
		for (int i = 0; i < actual_point->adjCount; i++)
		{
			if (actual_point->adjList[i].isClosed())
				continue;

			GraphNode *adjPoint = actual_point->adjList[i].node;
			const double alt = actual_point->dist + actual_point->adjList[i].distance;
			if (adjPoint->dist > alt)
			{
				adjPoint->dist = alt;
				adjPoint->prev = actual_point;
				dijMinHeap.minHeapifyR(adjPoint->minHeapIndex);
			}
		}
	}
}

GraphNodeWithSymmetric::GraphNodeWithSymmetric(int x, int y, const char *name,
	GraphNodeWithSymmetric *symmetric, AdjNode *adjList, int adjCount)
: GraphNode(x, y, name, adjList, adjCount), symmetric(symmetric)
{
}

GraphNodeIterator::GraphNodeIterator()
{
	rewind();
}

void GraphNodeIterator::rewind()
{
	i = 0;
}

GraphNode *GraphNodeIterator::next()
{
	return (i < NODECOUNT) ? allNodes[i++] : NULL;
}

GraphNode *findNodeByName(const char *name)
{
	for (int i = 0; i < NODECOUNT; i++)
	{
		GraphNode *n = allNodes[i];
		if (n->name != NULL && strcmp(n->name, name) == 0)
			return n;
	}

	return NULL;
}

void showNodes()
{
	for (int i = 0; i < NODECOUNT; i++)
	{
		GraphNode *n = allNodes[i];
		printf("- punto #%d %s [%.0f, %.0f]: path_length: %d\n -> ",
		       i, n->printableName(), n->pos.x, n->pos.y, measurePath(n));

		if (n->isIsolated())
		{
			printf("(punto isolato)\n");
		}
		else
		{
			for (int j = 0; j < n->adjCount; j++)
			{
				GraphNode *o = n->adjList[j].node;

				if (j != 0)
					printf(", ");

				printf("%s [%.0f, %.0f]", o->printableName(), o->pos.x, o->pos.y);

				if (n->adjList[j].isClosed())
					printf(" (chiuso)");
			}
			printf("\n");
		}
	}
}

void openAllEdges()
{
	for (int i = 0; i < NODECOUNT; i++)
		allNodes[i]->openAllEdges();
}

void openEdgesToBeOpened()
{
	for (int i = 0; i < NODECOUNT; i++)
		allNodes[i]->openEdgesToBeOpened();
}

GraphNode* findNearestPoint(const Point &pos)
{
	double distance_new, distance_old = DBL_MAX;
	GraphNode *min = NULL;

	for (int i = 0; i < NODECOUNT; i++)
	{
		GraphNode *n = allNodes[i];

		// Considera solo vertici non isolati
		if (n->isIsolated() == false)
		{
			distance_new = distance2(pos, n->pos);
			if (distance_new < distance_old)
			{
				distance_old = distance_new;
				min = n;
			}
		}
	}

	return min;
}

void closePointsInDirection(double dir)
{
	double max_size_along_x_axis = (DIM_H_BACK > DIM_H_FRONT) ? DIM_H_BACK : DIM_H_FRONT;
	double our_radius = sqrtf(max_size_along_x_axis*max_size_along_x_axis + (double)DIM_SIDE*DIM_SIDE);
	Point our_pos, min_enemy_pos, max_enemy_pos;

	our_pos = get_pos();

	double c = cos(TO_RADIANS(dir));
	double s = sin(TO_RADIANS(dir));

	min_enemy_pos.x = our_pos.x + (our_radius + ENEMY_RADIUS) * c;
	min_enemy_pos.y = our_pos.y + (our_radius + ENEMY_RADIUS) * s;
	max_enemy_pos.x = our_pos.x + DIST_TRESHOLD * c;
	max_enemy_pos.y = our_pos.y + DIST_TRESHOLD * s;

	const Capsule enemy_area(min_enemy_pos, max_enemy_pos, ENEMY_RADIUS);

	printf("Stringa magica chiusura archi: <%.0f,%.0f c %.0f,%.0f %.0f,%.0f %u>\n",
	       our_pos.x, our_pos.y, min_enemy_pos.x, min_enemy_pos.y, max_enemy_pos.x, max_enemy_pos.y, ENEMY_RADIUS);

	for (int i = 0; i < NODECOUNT; i++)
		allNodes[i]->closeEdgesIntersecting(enemy_area, our_radius, our_pos, TO_RADIANS(dir));
}

int getPath(GraphNode *gp)
{
	//printf("Generating Path to: %s\n", gp->printableName());

	GraphNode *sp = findNearestPoint(get_pos()); //start point
	if (sp == gp)
	{
		//printf("Path: cappio di vertice %s", gp->printableName());
		path[0] = gp;
		return 1;
	}

	// Eseguiamo Dijkstra passando l'array "path" (che al momento non stiamo
	// utilizzando) come zona di memoria utilizzabile durante i calcoli
	GraphNode::dijkstra(sp, path);

	if (gp->prev == NULL)
		return IMPOSSIBLE;

	// Ricostruisci percorso (deve essere capovolto)
	int pos = NODECOUNT;
	while (gp->prev != NULL)
	{
		path[--pos] = gp;
		gp = gp->prev;
	}

	int dim = NODECOUNT - pos;
	if (dim > 0)
		memmove(path, path + pos, sizeof(GraphNode*) * dim);

	return dim;
}

static void printPath(int dim, const Point *appendPoint)
{
	const Point init_pos = get_pos();

	if (dim == 0 && appendPoint == NULL)
	{
		printf("Il path generato non contiene vertici!\n");
		return;
	}

	printf("Stringa magica path generato: <p %.0f,%.0f", init_pos.x, init_pos.y);

	if (appendPoint == NULL)
		appendPoint = &path[--dim]->pos;

	for (int i = 0; i < dim; i++)
		printf(" %.0f,%.0f", path[i]->pos.x, path[i]->pos.y);

	printf(" t %.0f,%.0f>\n", appendPoint->x, appendPoint->y);
}

int measurePath(GraphNode *gp)
{
	int dim = getPath(gp);

	if (dim == IMPOSSIBLE)
		return IMPOSSIBLE;

	Point prev_pos;
	prev_pos = get_pos();

	double path_length = 0;
	for (int i = 0; i < dim; i++)
	{
		path_length += distance(prev_pos, path[i]->pos);
		prev_pos = path[i]->pos;
	}

	return (int)path_length;
}

int measurePathXY(const Point &pos)
{
	int dim = getPath(findNearestPoint(pos));

	if (dim == IMPOSSIBLE)
		return IMPOSSIBLE;

	Point prev_pos;
	prev_pos = get_pos();
	
	double path_length = 0;
	for (int i = 0; i < dim - 1; i++)
	{
		path_length += distance(prev_pos, path[i]->pos);
		prev_pos = path[i]->pos;
	}

	path_length += distance(prev_pos, pos);
	return (int)path_length;
}

int doPath(GraphNode *gp, bool backward)
{
	int dim = getPath(gp);
	if (dim == IMPOSSIBLE)
	{
		printf("Path Impossibile!\n");
		return IMPOSSIBLE;
	}

	printPath(dim, NULL);

	heading_to(path[0]->pos, backward ? HEADING_BACK : HEADING_FRONT);
	if(wait(MotionEvent()) == MOTOR_LOCKED)
		return IMPOSSIBLE;

	double obstacle_dir;
	get_pos(&obstacle_dir);

	if (backward)
	{
		obstacle_dir += 180;
		if (obstacle_dir > 180)
			obstacle_dir -= 360;
	}

	if (check_obstacle_direction(&obstacle_dir))
	{
		printf("do_path: IMPOSSIBLE perche' c'e' un ostacolo davanti (dir = %f)\n", obstacle_dir);
		closePointsInDirection(obstacle_dir);
		return IMPOSSIBLE;
	} 

	for(int i = 0; i < dim; i++)
		line_to_point(path[i]->pos, backward); 

	return POSSIBLE;

}

int doPathXY(const Point &pos, bool backward)
{
	int dim = getPath(findNearestPoint(pos));
	if (dim == IMPOSSIBLE)
	{
		printf("Path Impossibile!\n");
		return IMPOSSIBLE;
	}

	printPath(dim - 1, &pos);

	heading_to(path[0]->pos, backward ? HEADING_BACK : HEADING_FRONT);
	if(wait(MotionEvent()) == MOTOR_LOCKED)
		return IMPOSSIBLE;

	double obstacle_dir;
	get_pos(&obstacle_dir);
	if (backward)
	{
		obstacle_dir += 180;
		if (obstacle_dir > 180)
			obstacle_dir -= 360;
	}

	if (check_obstacle_direction(&obstacle_dir))
	{
		printf("do_path: IMPOSSIBLE perche' c'e' un ostacolo davanti (dir = %f)\n", obstacle_dir);
		closePointsInDirection(obstacle_dir);
		return IMPOSSIBLE;
	}

	for(int i = 0; i < dim - 1; i++)
		line_to_point(path[i]->pos, backward);
	line_to_point(pos, backward);

	return POSSIBLE;
}

class PathCutter
{
	public:
		PathCutter(const Circle &cutArea, const Point &initialPos)
		: cutArea(cutArea), prevPos(initialPos)
		{
			cut = cutArea.containsPoint(initialPos);
		}

                void addPoint(const Point &p, bool backward = false)
		{
			if (cut) // Ignora punti successivi al punto di taglio
				return;

			if (cutArea.containsPoint(p))
			{
				const Segment s(prevPos, p);
				Point cutPoint;
				cut = true;
				if (s.intersectAndReturnNearestPoint(cutArea, &cutPoint))
                                    line_to_point(cutPoint, backward);
				else
                                    line_to_point(prevPos, backward);
			}
			else
			{
                            line_to_point(prevPos, backward);
                            prevPos = p;
			}
		}

	private:
		Circle cutArea;
		Point prevPos;
		bool cut;
};


bool get_heading_for_go_with_offset(const Point &target, float offset_x, float offset_y, float *target_h)
{
    double theta;
    Point r_pos = get_pos(&theta);
    theta = TO_RADIANS(theta);

    bool reverse_dir = (offset_x < 0);
    float adj_offset_x, adj_offset_y;

    if (reverse_dir)
        {
            adj_offset_x = -offset_x;
            adj_offset_y = -offset_y;
        }
    else
        {
            adj_offset_x = offset_x;
            adj_offset_y = offset_y;
        }

    float target_dx = target.x - r_pos.x, target_dy = target.y - r_pos.y;
    float target_dist = sqrt(target_dx * target_dx + target_dy * target_dy);

    float acos_argument = -adj_offset_y / target_dist;

    if (fabs(acos_argument) > 1)
        return false;

    float target_dir = normalize_angle((reverse_dir ? -HALF_PI : HALF_PI) + normalize_angle(atan2(target_dy, target_dx) - acos(acos_argument)));

    *target_h = target_dir;

    return true;
}


void rotate_to_point_with_offset(const Point &pos, int offset_x, int offset_y)
{
    float target_heading;
    if (get_heading_for_go_with_offset(pos, offset_x, offset_y, &target_heading))
        rotate_absolute(TO_DEGREES(target_heading));
}


int doPathWithOffset(const Point &pos, int offset_x, int offset_y, int go_with_offset_distance)
{
	bool backward = (offset_x < 0);
	int dim = getPath(findNearestPoint(pos));
	if (dim == IMPOSSIBLE)
	{
		printf("Path Impossibile!\n");
		return IMPOSSIBLE;
	}

	printPath(dim - 1, &pos);

	if (dim == 1)
	{
		heading_to_with_offset(pos, offset_x, offset_y);
	}
	else
	{
		if (backward)
			heading_to(path[0]->pos, HEADING_BACK);
		else
			heading_to(path[0]->pos, HEADING_FRONT);
	}

	if(wait(MotionEvent()) == MOTOR_LOCKED)
		return IMPOSSIBLE;

	double obstacle_dir;
	Point s_pos = get_pos(&obstacle_dir);

	if (backward)
	{
		obstacle_dir += 180;
		if (obstacle_dir > 180)
			obstacle_dir -= 360;
	}

	if (check_obstacle_direction(&obstacle_dir))
	{
		printf("do_path: IMPOSSIBLE perche' c'e' un ostacolo davanti (dir = %f)\n", obstacle_dir);
		closePointsInDirection(obstacle_dir);
		return IMPOSSIBLE;
	}

	PathCutter cutter(Circle(pos, go_with_offset_distance), s_pos);
	for(int i = 0; i < dim - 1; i++)
            cutter.addPoint(path[i]->pos, backward);
	cutter.addPoint(pos, backward);
	go_with_offset(pos, offset_x, offset_y);

	return POSSIBLE;
}

bool forward_to_point_with_retry(const Point &target, double threshold)
{
	AutoControllerParamsRestorer<DistanceControllerParameters> autorestorer_d;
	distance_slow.apply();
	set_obstacle_avoidance_nearonly(true);

	double angle;
	const Point start_pos = get_pos();
	Point robot_pos;
	threshold *= threshold;

	while (true)
	{
		// calcola direzione da controllare
		robot_pos = get_pos();
		
		// Se siamo entro 10 mm dal punto target usciamo per evitare atan2(0, 0)
		if(distance2(target, robot_pos) < 10*10)
		{
			set_obstacle_avoidance_nearonly(false);
			return true;
		}
		
		angle = TO_DEGREES(atan2(target.y - robot_pos.y, target.x - robot_pos.x));

		printf("forward_to_point_with_retry: controllo in direzione %f\n", angle);

		if (check_obstacle_direction(&angle) == false)
		{
			printf("Libero!\n");
			forward_to_point(target);
			if (wait(MotionEvent()) == PATH_DONE)
			{
				set_obstacle_avoidance_nearonly(false);
				return true;
			}
		}
		else
		{
			printf("Ostacolo\n");
			if(threshold < distance2(start_pos, robot_pos))
			{
				printf("forward_to_point_with_retry: esco dato che ho raggiunto la soglia minima\n");
				set_obstacle_avoidance_nearonly(false);
				return false;
			}
		}

		__delay_ms(500);
	}
}
