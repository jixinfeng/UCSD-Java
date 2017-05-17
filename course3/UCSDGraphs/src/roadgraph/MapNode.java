package roadgraph;

import java.util.HashSet;
import geography.GeographicPoint;

class MapNode {
	// Location of this node
	private GeographicPoint location;
	// A hashset of edges pointing out from this node
	private HashSet<MapEdge> outEdges;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		outEdges = new HashSet<MapEdge>();
	}
	
	public boolean addOutEdge(MapEdge edge) {
		// Add edge to outEdges if the new edge is pointing out from this node
		// and return true. Else return false
		if (edge.getFromLoc().equals(this.location)) {
			outEdges.add(edge);
			return true;
		} else {
			return false;
		}
	}
	
	// trivial getters and setters
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
