using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Node {
	public bool walkable;
	public Vector3 worldPosition;
	public int gridPosX;
	public int gridPosY;
	public List<Node> neighbours;
	public float cost=0;

	public Node(bool walkable, Vector3 worldPosition, int gridPosX, int gridPosY) {
		neighbours = new List<Node> ();
		this.walkable = walkable;
		this.worldPosition = worldPosition;
		this.gridPosX = gridPosX;
		this.gridPosY = gridPosY;
	}
}
