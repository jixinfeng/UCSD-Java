/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ArrayList;
import java.util.Set;
import java.util.PriorityQueue;

import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Store vertices/nodes and edges
	private HashMap<GeographicPoint, MapNode> vertices;
	private HashSet<MapEdge> edges;
	
	private double INFINITE = Double.POSITIVE_INFINITY;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (vertices.containsKey(location)){
			return false;
		} else {
			vertices.put(location, new MapNode(location));
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(
			GeographicPoint from, 
			GeographicPoint to, 
			String roadName,
			String roadType, 
			double length) throws IllegalArgumentException {
		if (!vertices.containsKey(from) || !vertices.containsKey(to)) {
			throw new IllegalArgumentException();
		} else if (roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException();
		} else {
			// create new edge
			MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
			// store new edge
			edges.add(newEdge);
			// link edge to its associated start node
			vertices.get(from).addOutEdge(newEdge);
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(
			GeographicPoint start,
			GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched)
	{
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		List<MapNode> queue = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		queue.add(vertices.get(start));
		visited.add(vertices.get(start));
		MapNode currNode;
		
		while (!queue.isEmpty()) {
			currNode = queue.remove(0);
			nodeSearched.accept(currNode.getLocation());
			if (currNode.getLocation().equals(goal)) {
				return parentMap2Path(start, goal, parentMap);
			}
			for (MapEdge outEdge: currNode.getOutEdges()) {
				MapNode neighbor = vertices.get(outEdge.getToLoc());
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor.getLocation(), currNode.getLocation());
					queue.add(neighbor);
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(
			GeographicPoint start, 
			GeographicPoint goal, 
			Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		Set<MapNode> q = new HashSet<MapNode>();
		HashMap<MapNode, Double> dist = new HashMap<MapNode, Double>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		for (MapNode node: vertices.values()) {
			q.add(node);
			dist.put(node, INFINITE);
		}
		
		dist.put(vertices.get(start), 0.0);
		while (!q.isEmpty()) {
			MapNode currNode = getClosestNode(q, dist);
			q.remove(currNode);
			nodeSearched.accept(currNode.getLocation());
			if (currNode.getLocation().equals(goal)) {
				break;
			}
			for (MapNode neighbor: getNeighborNodes(currNode)) {
				nodeSearched.accept(neighbor.getLocation());
				double newDist = dist.get(currNode) + getEdge(currNode, neighbor).getLength();
				if (newDist < dist.get(neighbor)) {
					dist.put(neighbor, newDist);
					parentMap.put(neighbor.getLocation(), currNode.getLocation());
				}
			}
		}
		return parentMap2Path(start, goal, parentMap); 
		
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, 
											 Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		Set<MapNode> q = new HashSet<MapNode>();
		HashMap<MapNode, Double> dist = new HashMap<MapNode, Double>();
		HashMap<MapNode, Double> est = new HashMap<MapNode, Double>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		for (MapNode node: vertices.values()) {
			q.add(node);
			dist.put(node, INFINITE);
			est.put(node, INFINITE);
		}
		
		dist.put(vertices.get(start), 0.0);
		est.put(vertices.get(start), goal.distance(start));
		while (!q.isEmpty()) {
			MapNode currNode = getClosestNode(q, dist, est);
			q.remove(currNode);
			nodeSearched.accept(currNode.getLocation());
			if (currNode.getLocation().equals(goal)) {
				break;
			}
			for (MapNode neighbor: getNeighborNodes(currNode)) {
				nodeSearched.accept(neighbor.getLocation());
				if (est.get(neighbor) == INFINITE) {
					est.put(neighbor, goal.distance(neighbor.getLocation()));
				}
				double newDist = dist.get(currNode) + getEdge(currNode, neighbor).getLength();
				if (newDist < dist.get(neighbor)) {
					dist.put(neighbor, newDist);
					parentMap.put(neighbor.getLocation(), currNode.getLocation());
				}
			}
		}
		return parentMap2Path(start, goal, parentMap); 
	}

	public List<MapNode> TSP(boolean geoDist) {
		HashMap<MapNode, HashMap<MapNode, Double>> pairwiseDists;
		if (!geoDist) {
			if (!isConnected()) {
				System.out.println("Not connected!");
				return null;
			}
			System.out.println("Connected!");
			pairwiseDists = calcPairwiseTravelDist();
		} else {
			pairwiseDists = calcPairwiseGeoDist();
		}
		List<MapNode> TSPTravel =  greedyTSP(pairwiseDists);
		double greedyTSPLen = calcTSPLength(TSPTravel, pairwiseDists);
		System.out.println("Graph order:");
		System.out.println(vertices.size());
		List<MapNode> twoOptTravel = TSPTravel;
		double twoOptTravelLen = greedyTSPLen;
		List<MapNode> bestTravel = TSPTravel;
		double bestTravelLen = greedyTSPLen;
		if (vertices.size() >= 4) {
			do {
				System.out.println("Found better travel!");
				System.out.println(bestTravelLen);
				bestTravelLen = twoOptTravelLen;
				bestTravel = twoOptTravel;
				twoOptTravel = twoOptIteration(bestTravel, pairwiseDists);
				twoOptTravelLen = calcTSPLength(twoOptTravel, pairwiseDists);
			} while (twoOptTravelLen < bestTravelLen);
		}
		return bestTravel;
	}
	
	private List<MapNode> twoOptIteration(List<MapNode> travel, HashMap<MapNode, HashMap<MapNode, Double>> dists) {
		List<MapNode> newTravel = new ArrayList<MapNode>(travel);
		double newTravelLen = INFINITE;
		List<MapNode> bestTravel = new ArrayList<MapNode>(travel);
		double bestTravelLen = calcTSPLength(travel, dists);
		for (int i = 1; i < travel.size() - 2; i++) {
			for (int j = 2; j < travel.size() - 1; j ++) {
				newTravel = twoOptSwap(travel, i, j);
				newTravelLen = calcTSPLength(newTravel, dists);
				if (newTravelLen < bestTravelLen) {
					bestTravel = newTravel;
					bestTravelLen = newTravelLen;
				}
			}
		}
		return bestTravel;
	}
	
	private List<MapNode> twoOptSwap(List<MapNode> path, int begin, int end) { //both inclusive
		if (begin < 1 || end >= path.size() - 1 || end <= begin) {
			return new ArrayList<MapNode>(path);
		}
		List<MapNode> swapPath = new ArrayList<MapNode>();
		for (int i = 0; i < begin; i++) {
			swapPath.add(path.get(i));
		}
		for (int i = end; i >= begin; i--) {
			swapPath.add(path.get(i));
		}
		for (int i = end + 1; i < path.size(); i++) {
			swapPath.add(path.get(i));
		}
		return swapPath;
	}
	
	private List<MapNode> greedyTSP(HashMap<MapNode, HashMap<MapNode, Double>> dists) {
		if (vertices == null || vertices.size() == 0) {
			return null;
		}
		List<MapNode> travel = new ArrayList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		MapNode nextNode = pickInitNode();
		while (nextNode != null) {
			travel.add(nextNode);
			visited.add(nextNode);
			nextNode = pickGreedyNextNode(nextNode, visited, dists);
		}
		return travel;
	}
	
	private HashMap<MapNode, HashMap<MapNode, Double>> calcPairwiseGeoDist() {
		HashMap<MapNode, HashMap<MapNode, Double>> pairwiseGeoDist = new HashMap<MapNode, HashMap<MapNode, Double>>();
		for (MapNode fromNode: vertices.values()) {
			HashMap<MapNode, Double> geoDists = new HashMap<MapNode, Double>();
			for (MapNode toNode: vertices.values()) {
				double geoDist;
				if (fromNode == toNode) {
					continue;
				} else {
					GeographicPoint fromPoint = fromNode.getLocation();
					GeographicPoint toPoint = toNode.getLocation();
					geoDists.put(toNode, fromPoint.distance(toPoint));
				}
			}
			pairwiseGeoDist.put(fromNode, geoDists);
		}
		return pairwiseGeoDist;
	}
	
	private HashMap<MapNode, HashMap<MapNode, Double>> calcPairwiseTravelDist() {
		HashMap<MapNode, HashMap<MapNode, Double>> pairwiseTravelDist = new HashMap<MapNode, HashMap<MapNode, Double>>();
		for (MapNode fromNode: vertices.values()) {
			HashMap<MapNode, Double> travelDists = new HashMap<MapNode, Double>();
			for (MapNode toNode: vertices.values()) {
				double travelDist;
				if (fromNode == toNode) {
					continue;
				} else if (getNeighborNodes(fromNode).contains(toNode)) {
					MapEdge connectingEdge = getEdge(fromNode, toNode);
					travelDist = connectingEdge.getLength();
					travelDists.put(toNode, travelDist);
				} else {
					List<GeographicPoint> travelPath = aStarSearch(fromNode.getLocation(), toNode.getLocation());
					travelDist = calcPathLength(travelPath);
					travelDists.put(toNode, travelDist);
				}
			}
			pairwiseTravelDist.put(fromNode, travelDists);
		}
		return pairwiseTravelDist;
	}
	
	private Double calcPathLength(List<GeographicPoint> path) {
		if (path == null || path.size() <= 1) {
			return (double) 0;
		}
		double pathLength = 0.0;
		for (int i = 0; i < path.size() - 1; i++) {
			MapNode currFrom = vertices.get(path.get(i));
			MapNode currTo = vertices.get(path.get(i + 1));
			MapEdge currEdge = getEdge(currFrom, currTo);
			pathLength += currEdge.getLength();
		}
		return pathLength;
	}
	
	private MapNode pickInitNode() {
		return vertices.values().iterator().next();
	}
	
	private MapNode pickGreedyNextNode(MapNode currNode, Set<MapNode> visited, HashMap<MapNode, HashMap<MapNode, Double>> dists) {
		GeographicPoint currNodeLoc = currNode.getLocation(); 
		MapNode currNearest = null;
		double currNearestDist = INFINITE;
		for (MapNode node: vertices.values()) {
			if (visited.contains(node) || node == currNode) {
				continue;
			}
			double dist = node.getLocation().distance(currNodeLoc);
			if (dist < currNearestDist) {
				currNearest = node;
				currNearestDist = dist;
			}
		}
		return currNearest;
	}
	
	private boolean isConnected() {
		MapNode currNode = pickInitNode();
		Set<MapNode> visited = new HashSet<MapNode>();
		List<MapNode> queue = new LinkedList<MapNode>();
		
		queue.add(currNode);
		visited.add(currNode);
		
		while(!queue.isEmpty()) {
			currNode = queue.remove(0);
			for (MapEdge outEdge: currNode.getOutEdges()) {
				MapNode neighbor = vertices.get(outEdge.getToLoc());
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					queue.add(neighbor);
				}
			}
		}
		return visited.size() == vertices.size();
	}

	private Double calcTSPLength(List<MapNode> travel, HashMap<MapNode, HashMap<MapNode, Double>> dists) {
		if (travel == null || travel.size() <= 1) {
			return (double) 0;
		}
		double TSPLength = 0.0;
		for (int i = 0; i < travel.size(); i++) {
			MapNode currFrom = travel.get(i);
			MapNode currTo = travel.get((i + 1) % travel.size());
			TSPLength += dists.get(currFrom).get(currTo);
		}
		return TSPLength;
	}
	
	private List<MapNode> getNeighborNodes(MapNode node) {
		List<MapNode> neighbors = new LinkedList<MapNode>();
		for (MapEdge outEdge: node.getOutEdges()) {
			neighbors.add(vertices.get(outEdge.getToLoc()));
		}
		return neighbors;
	}
	
	private MapEdge getEdge(GeographicPoint fromLoc, GeographicPoint toLoc) {
		if (!vertices.containsKey(fromLoc) || ! vertices.containsKey(toLoc)) {
			return null;
		} else {
			MapNode fromNode = vertices.get(fromLoc);
			MapNode toNode = vertices.get(toLoc);
			return getEdge(fromNode, toNode);
		}
	}
	
	private MapEdge getEdge(MapNode fromNode, MapNode toNode) {
		Set<MapEdge> outEdges = fromNode.getOutEdges();
		for (MapEdge outEdge: outEdges) {
			if (vertices.get(outEdge.getToLoc()) == toNode) {
				return outEdge;
			}
		}
		return null;
	}
	
	private MapNode getClosestNode(Set<MapNode> q, HashMap<MapNode, Double> dist) {
		if (q.isEmpty()) {
			return null;
		} 
		double minDist = INFINITE;
		MapNode closestNode = null;
		for (MapNode node: q) {
			if (dist.get(node) < minDist) {
				minDist = dist.get(node);
				closestNode = node;
			}
		}
		return closestNode;
	}
	
	private MapNode getClosestNode(
			Set<MapNode> q, 
			HashMap<MapNode, Double> dist,
			HashMap<MapNode, Double> est) {
		if (q.isEmpty()) {
			return null;
		} 
		double minDist = INFINITE;
		MapNode closestNode = null;
		for (MapNode node: q) {
			if (dist.get(node) + est.get(node) < minDist) {
				minDist = dist.get(node) +est.get(node);
				closestNode = node;
			}
		}
		return closestNode;
	}

	// Convert parent map into path from start to goal as linked list
	// Return a linked list with only goal node if parent map is empty
	// Currently this method assume the parent map has all required 
	// information for building the path
	// Only been tested with BST but should works fine with Dijkstra 
	// and A* with minimal revisions
	private List<GeographicPoint> parentMap2Path(
			GeographicPoint start,
			GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.add(goal);
		if (parentMap.isEmpty()) {
			return path;
		}
		GeographicPoint currPoint = goal;
		GeographicPoint parentPoint;
		while (!currPoint.equals(start)) {
			parentPoint = parentMap.get(currPoint);
			path.add(0, parentPoint);
			currPoint = parentPoint;
		}
		return path;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		///*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println("TSP Test using simpletest");
		simpleTestMap.TSP(true);
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		//*/
		
		System.out.println("TSP Test using utc");
		testMap.TSP(true);
		
		
		/* Use this code in Week 3 End of Week Quiz */
		///*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		System.out.println("TSP Test using quiz map");
		theMap.TSP(true);
		
		MapGraph hollywood_s = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/hollywood_small.map", hollywood_s);
		System.out.println("DONE.");

		System.out.println("TSP Test using hollywood_s map");
		hollywood_s.TSP(true);
		
		MapGraph hollywood_l = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/hollywood_large.map", hollywood_l);
		System.out.println("DONE.");

		System.out.println("TSP Test using hollywood_b map");
		hollywood_l.TSP(true);

	}
	
}
