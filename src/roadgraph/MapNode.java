package roadgraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{

	private GeographicPoint location;
	private HashSet<MapEdge> edges;
	private double distance;
	private double actualDistance;

	public MapNode(GeographicPoint location) {
		super();
		this.location = location;
		this.edges = new HashSet<MapEdge>();
		this.distance = 0.0;
		this.actualDistance = 0.0;
		
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public double getDistance() {
		return this.distance;
	}

	public double getActualDistance() {
		return this.actualDistance;
	}

	public Set<MapNode> getNeighbours() {
		Set<MapNode> neighbours = new HashSet<MapNode>();
		for (MapEdge e : edges) {
			neighbours.add(e.getOtherNode(this));
		}

		return neighbours;
	}

	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	public Set<MapEdge> getEdges() {
		return edges;
	}

	@Override
	public boolean equals(Object o) {
		if (!(o instanceof MapNode) || o == null)
			return false;

		MapNode node = (MapNode) o;

		return this.location.equals(node.location);
	}

	@Override
	public int compareTo(MapNode o) {
		return ((Double)this.getDistance()).compareTo(((Double)o.getDistance()));
	}


}
