package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	
	private GeographicPoint location;
	private List<MapEdge> neighbours;
	
	public MapNode(GeographicPoint location, List<MapEdge> neighbours) {
		super();
		this.location = location;
		this.neighbours = neighbours;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public List<MapEdge> getNeighbours() {
		return neighbours;
	}

	public void setNeighbours(List<MapEdge> neighbours) {
		this.neighbours = neighbours;
	}
	
}
