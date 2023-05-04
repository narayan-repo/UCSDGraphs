package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double distance;

	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double distance) {
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.distance = distance;
	}

	public GeographicPoint getStart() {
		return from;
	}

	public void setStart(GeographicPoint from) {
		this.from = from;
	}

	public GeographicPoint getEnd() {
		return to;
	}

	public void setEnd(GeographicPoint to) {
		this.to = to;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

}
