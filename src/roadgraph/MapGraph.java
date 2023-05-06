/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	private Map<GeographicPoint, MapNode> vertices;
	private Set<MapEdge> edges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		this.vertices = new HashMap<GeographicPoint, MapNode>();
		this.edges = new HashSet<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return vertices.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		Set<GeographicPoint> vSet = new HashSet<GeographicPoint>();
		vSet.addAll(vertices.keySet());
		return vSet;
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edges.size();
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location The location of the intersection
	 * @return true if a node was added, false if it was not (the node was already
	 *         in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null || vertices.containsKey(location)) {
			return false;
		}

		vertices.put(location, new MapNode(location));
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from     The starting point of the edge
	 * @param to       The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length   The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been added as
	 *                                  nodes to the graph, if any of the arguments
	 *                                  is null, or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		MapNode f = vertices.get(from);
		MapNode e = vertices.get(to);

		if (f == null || e == null || length < 0)
			throw new IllegalArgumentException("Invalid vertice location");

		MapEdge edge = new MapEdge(f, e, roadName, roadType, length);
		edges.add(edge);

		f.addEdge(edge);

	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		if (!checkLocationValidity(start, goal)) {
			return null;
		}

		Queue<MapNode> queue = new LinkedList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();

		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		MapNode next = null;

		queue.add(startNode);
		visited.add(startNode);

		while (!queue.isEmpty()) {

			next = queue.remove();

			nodeSearched.accept(next.getLocation());

			if (next.equals(goalNode)) {
				break;
			}

			for (MapNode neighbour : next.getNeighbours()) {
				if (!visited.contains(neighbour)) {
					visited.add(neighbour);
					parent.put(neighbour, next);
					queue.add(neighbour);
				}
			}
		}
		return reconstructPath(parent, startNode, goalNode, next.equals(goalNode));

	}

	private List<GeographicPoint> reconstructPath(HashMap<MapNode, MapNode> parent, MapNode startNode, MapNode goalNode,
			boolean pathAvailable) {

		if (!pathAvailable) {
			System.out.println("No Path available");
			return null;
		}

		LinkedList<GeographicPoint> route = new LinkedList<GeographicPoint>();
		MapNode current = goalNode;

		while (!current.equals(startNode)) {
			route.addFirst(current.getLocation());
			current = parent.get(current);
		}

		route.addFirst(startNode.getLocation());
		return route;
	}

	private boolean checkLocationValidity(GeographicPoint p1, GeographicPoint p2) {

		if (p1 == null || p2 == null) {
			throw new NullPointerException("Start or Goal node is null");
		}

		if (vertices.get(p1) == null || vertices.get(p2) == null) {
			System.out.println("Start node or Goal node doesn't exist");
			return false;
		}

		return true;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		return searchOnWeightedGraph(start, goal, nodeSearched,(a, b) -> 0.0);

	}

	private List<GeographicPoint> searchOnWeightedGraph(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, BiFunction<MapNode, MapNode, Double> f) {
		
		if (!checkLocationValidity(start, goal)) {
			return null;
		}

		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();

		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		MapNode next = null;

		initializeDistances();
		
		startNode.setDistance(0.0f);
		startNode.setActualDistance(0.0f);

		queue.add(startNode);

		while (!queue.isEmpty()) {

			next = queue.poll();
			if (!visited.contains(next)) {
				visited.add(next);

				nodeSearched.accept(next.getLocation());

				if (next.equals(goalNode)) {
					break;
				}

				HashMap<MapNode, Double> distanceMap = getDistanceMap(next);
				
				
				for (MapNode neighbour : next.getNeighbours()) {
					if(!visited.contains(neighbour)) {
						double distanceOfNode = next.getActualDistance() + distanceMap.get(neighbour);
						if(distanceOfNode < neighbour.getActualDistance()) {
							neighbour.setActualDistance(distanceOfNode);
							distanceOfNode += f.apply(neighbour, goalNode);
							neighbour.setDistance(distanceOfNode);
							parent.put(neighbour, next);
							queue.offer(neighbour);
						}
						
					}
				}
			}
		}
		
		System.out.println("Visited: " + visited.size());
		
		return reconstructPath(parent, startNode, goalNode, next.equals(goalNode));
	}

	private HashMap<MapNode, Double> getDistanceMap(MapNode next) {
		HashMap<MapNode, Double> distanceMap = new HashMap<>();
		for(MapEdge edge: next.getEdges()) {
			distanceMap.put(edge.getTo(), edge.getDistance());
		}
		return distanceMap;
	}

	private void initializeDistances() {
		for (MapNode m : vertices.values()) {
			m.setDistance(Double.MAX_VALUE);
			m.setActualDistance(Double.MAX_VALUE);
		}
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		return searchOnWeightedGraph(start, goal, nodeSearched, (a,b) -> a.getLocation().distance(b.getLocation()));
	}

	public static void main(String[] args) {

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}

}
