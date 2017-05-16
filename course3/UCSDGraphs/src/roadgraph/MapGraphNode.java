/**
 * 
 */
package roadgraph;

import java.util.LinkedList;
import java.util.List;

import week3example.MazeNode;

/**
 * @author jfeng
 *
 */
class MapGraphNode {
	private List<MapGraphNode> neighbors;
	
	private int row;
	private int column;
	private char displayChar;
	
	public static final char EMPTY = '-';
	public static final char PATH = 'o';
	public static final char START = 'S';
	public static final char GOAL = 'G';
	
	public void setDisplayChar(char displayChar) {
		this.displayChar = displayChar;
	}
	
	public MapGraphNode(int row, int col)
	{
		this.row = row;
		this.column = col;
		neighbors = new LinkedList<MapGraphNode>();
		displayChar = EMPTY;
	}
	
	public void addNeighbor(MapGraphNode neighbor) 
	{
		neighbors.add(neighbor);
	}
	
	public List<MapGraphNode> getNeighbors() {
		return neighbors;
	}

	/**
	 * @return the row
	 */
	public int getRow() {
		return row;
	}

	/**
	 * @return the column
	 */
	public int getColumn() {
		return column;
	}
}
