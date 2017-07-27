/*
 * routing.h
 */


#ifndef __ROUTING_H
#define __ROUTING_H

#include "defines.h"
#include "geometry.h"

#include <stdbool.h>
#include <float.h>

#define IMPOSSIBLE	-1
#define POSSIBLE	1

#define AGE			8

/* Gli oggetti GraphNode e GraphNodeWithSymmetric vengono istanziati
 * nei file generati automaticamente da graph2cpp a partire da routing.graph.
 * Nota: NON c'è motivo di istanziare oggetti GraphNode e GraphNodeWithSymmetric
 * direttamente tramite codice!! */
class GraphNode
{
	friend int getPath(GraphNode *gp);
	friend void showNodes();

	public:
		// Elemento nella lista di adiacenza
		struct AdjNode
		{
			// Questi due campi vengono valorizzati a compile time
			// dal codice python di generazione del grafo
			GraphNode *const node;
			const double distance; // distanza euclidea

			// I seguenti campi sono invece valorizzati a runtime,
			// per memorizzare informazioni sulla chiusura dovuta
			// alla presenza di ostacoli
			bool closed_obstacle;
			int ct; // istante in cui e' stato chiuso il punto

			// Controllo isolamento temporaneo a causa di presenza di ostacoli
			void open();
			void close();
			bool isClosed() const { return closed_obstacle; }
			bool isToBeOpened() const { return closed_obstacle && (game_timer - ct) > AGE; }
		};

		GraphNode(int x, int y, const char *name, AdjNode *adjList, int adjCount);

		// Controllo isolamento temporaneo a causa di presenza di ostacoli
		void openAllEdges();
		void openEdgesToBeOpened();
		void closeAllEdges();
		void closeEdgesIntersecting(const Capsule &capsule, double edge_radius, const Point &robot_pos, double obstacle_dir);

		// È un nodo isolato (ovvero di grado zero)?
		bool isIsolated() const { return (adjCount == 0); }

		// Restituisce la label, se presente, oppure una stringa segnaposto
		const char* printableName() const { return name ?: "no-label"; }

		const Point pos;	// coordinate del nodo
		const char *name;	// nome del nodo oppure NULL

		// Gli oggetti GraphNode non possono essere copiati
		GraphNode(const GraphNode&) = delete;
		GraphNode &operator=(const GraphNode&) = delete;

	private:
		struct DijkstraMinHeap
		{
			DijkstraMinHeap(GraphNode *sp, GraphNode **scratchpad);
			bool isEmpty() const;
			GraphNode *extractMin();
			void minHeapify(unsigned int i);
			void minHeapifyR(unsigned int i);
			bool isLessThan(unsigned int i, unsigned int j);
			void swap(unsigned int i, unsigned int j);

			GraphNode **heap;
			unsigned int size;
		};

		static void dijkstra(GraphNode *sp, GraphNode **scratchpad);

		// Lista di adiacenza (e relativo numero di elementi)
		AdjNode *adjList;
		int adjCount;

		// Variabili usate per la computazione del cammino tramite Dijkstra
		GraphNode *prev;	// nodo di provenienza
		double dist;		// distanza del punto dalla destinazione lungo il cammino di ricerca
		int minHeapIndex;	// posizione del nodo nel min-heap di appoggio
};

class GraphNodeWithSymmetric : public GraphNode
{
	public:
		GraphNodeWithSymmetric(int x, int y, const char *name,
			GraphNodeWithSymmetric *symmetric, AdjNode *adjList, int adjCount);

		GraphNodeWithSymmetric *const symmetric;
};

class GraphNodeIterator
{
	public:
		GraphNodeIterator();

		void rewind();
		GraphNode *next();

	private:
		int i;
};

/* Per ogni nodo dotato di nome, in questo file viene definita una variabile
 * Graph::NOMELABEL di tipo GraphNode oppure GraphNodeWithSymmetric */
#include "graph_definitions.gen"

GraphNode *findNodeByName(const char *name);
void showNodes();
void openAllEdges();
void openEdgesToBeOpened();
GraphNode* findNearestPoint(const Point &pos);
void closePointsInDirection(double dir);

int getPath(GraphNode *gp);
int measurePath(GraphNode *gp); // lunghezza del cammino da percorrere oppure IMPOSSIBLE
int measurePathXY(const Point &pos); // lunghezza del cammino da percorrere oppure IMPOSSIBLE

void rotate_to_point_with_offset(const Point &pos, int offset_x, int offset_y);

int doPath(GraphNode *gp, bool backward = false);		// execute the path
int doPathXY(const Point &pos, bool backward = false);		// go to given point
int doPathWithOffset(const Point &pos, int offset_x, int offset_y, int go_with_offset_distance);

bool forward_to_point_with_retry(const Point &target, double threshold = DBL_MAX);

#endif
