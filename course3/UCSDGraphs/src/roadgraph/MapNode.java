package roadgraph;

import java.util.HashSet;
import geography.GeographicPoint;

class MapNode {
	private GeographicPoint location;
	private HashSet<MapEdge> outEdges;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		outEdges = new HashSet<MapEdge>();
	}
	
	public boolean addOutEdge(MapEdge edge) {
		if (edge.getFromLoc().equals(this.location)) {
			outEdges.add(edge);
			return true;
		} else {
			return false;
		}
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public HashSet<MapEdge> getOutEdges() {
		return outEdges;
	}
}
