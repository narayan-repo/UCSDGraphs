package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private MapNode from;
	private MapNode to;
	private String roadName;
	private String roadType;
	private double distance;

	public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double distance) {
		super();
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.distance = distance;
	}

	public MapNode getFrom() {
		return from;
	}

	public MapNode getTo() {
		return to;
	}

	public GeographicPoint getFromLoc() {
		return from.getLocation();
	}

	public GeographicPoint getToLoc() {
		return to.getLocation();
	}

	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getDistance() {
		return distance;
	}

	public MapNode getOtherNode(MapNode node) {
		if (node.equals(to)) {
			return from;
		} else if (node.equals(from)) {
			return to;
		} else {
			throw new IllegalArgumentException("Invalid node");
		}
	}

}
