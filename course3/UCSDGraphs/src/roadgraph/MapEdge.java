package roadgraph;

import geography.GeographicPoint;

class MapEdge {
	// two GeographicPoints storing the from and to locations of an edge
	private GeographicPoint fromLoc;
	private GeographicPoint toLoc;
	// Store road name/type/length
	private String roadName;
	private String roadType;
	private double length;
	
	public MapEdge(
			GeographicPoint fromLoc, 
			GeographicPoint toLoc, 
			String roadName, 
			String roadType, 
			double length) {
		this.fromLoc = fromLoc;
		this.toLoc = toLoc;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	// Methods listed below are just trivial getters and setters
	public GeographicPoint getFromLoc() {
		return fromLoc;
	}

	public void setFromLoc(GeographicPoint fromLoc) {
		this.fromLoc = fromLoc;
	}

	public GeographicPoint getToLoc() {
		return toLoc;
	}

	public void setToLoc(GeographicPoint toLoc) {
		this.toLoc = toLoc;
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

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}
}
