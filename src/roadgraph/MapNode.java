package roadgraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode {

	private GeographicPoint location;
	private HashSet<MapEdge> edges;

	public MapNode(GeographicPoint location) {
		super();
		this.location = location;
		this.edges = new HashSet<MapEdge>();
	}

	public GeographicPoint getLocation() {
		return location;
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

}
