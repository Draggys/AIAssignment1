using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Grid : MonoBehaviour {

	public Transform player;
	public LayerMask unwalkableMask;
	public Vector2 gridWorldSize;
	public float nodeRadius;
	public Node[,] grid;
	public MapLoader mapLoader=null;

	public float nodeDiameter;
	public int gridSizeX;
	public int gridSizeY;

	public MapData mapData;

	public int neighbourhood;

	void Start () {
		MapLoader mapLoader = new MapLoader (new Vector2(20, 20), 0.5f);
		mapData = mapLoader.LoadMap ("A", "endPos", "startPos");

		nodeDiameter = mapData.nodeRadius * 2;
		gridWorldSize = mapData.gridWorldSize;
		// #nodes that can fit into the x-space
		gridSizeX = Mathf.RoundToInt (gridWorldSize.x / nodeDiameter);
		gridSizeY = Mathf.RoundToInt (gridWorldSize.y / nodeDiameter);
		CreateGrid ();
		
		foreach (Node node in grid) {
			if(neighbourhood == 4)
				FillNeighbourhood4 (node);
			if(neighbourhood == 8)
				FillNeighbourhood8 (node);
			if(neighbourhood == 16)
				FillNeighbourhood16 (node);
		}

		Node n = grid [4, 3];
		print (n.gridPosX + ", " + n.gridPosY + " Count: " + n.neighbours.Count);

	}

	public Node getNode(int gridPosX, int gridPosY) {
		return grid [gridPosX, gridPosY];
	}

	private bool validIndex(int x, int y) {
		if (0 <= x && x <= gridSizeX) {
			if(0 <= y && y <= gridSizeY) {
				return true;
			}
		}
		return false;
	}

	void CreateGrid() {
		grid = new Node[gridSizeX, gridSizeY];
		Vector3 worldTopLeft = transform.position - Vector3.right * gridWorldSize.x / 2
			+ Vector3.forward * gridWorldSize.y / 2;

		for (int x = 0; x < gridSizeX; x++) {
			for(int y = 0; y < gridSizeY; y++) {
				Vector3 worldPoint = worldTopLeft + Vector3.right * (y * nodeDiameter + nodeRadius)
					- Vector3.forward * (x * nodeDiameter + nodeRadius);
				bool walkable = mapData.walkable[x ,y];
				grid[y, x] = new Node(walkable, worldPoint, y, x);
			}
		}
		/*
		grid = new Node[gridSizeX, gridSizeY];
		Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2
			- Vector3.forward * gridWorldSize.y / 2;
		
		// Loop through all nodes to do collision check, see if walkable or not
		for (int x = 0; x < gridSizeX; x++) {
			for(int y = 0; y < gridSizeY; y++) {
				Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius)
					+ Vector3.forward * (y * nodeDiameter + nodeRadius);
				bool walkable = mapData.walkable[x ,y];
				grid[x, y] = new Node(walkable, worldPoint, x, y);
			}
		}
		*/
	}
	
	// Find node that player is currently standing on 
	// I.e convert a world position into a grid coordinate
	public Node NodeFromWorldPoint(Vector3 worldPosition) {
		float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
		float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;

		percentX = Mathf.Clamp01 (percentX);
		percentY = Mathf.Clamp01 (percentY);

		int x = Mathf.RoundToInt ((gridSizeX - 1) * percentX);
		int y = Mathf.RoundToInt ((gridSizeY - 1) * percentY);


		return grid[x, gridSizeY - 1 - y];
		/*
		// convert world position into a percentage for the x and why coordinate
		//  how far along the grid it is
		// far left 0, middle .5, right 1
	
		// add half grid world size then divide with whole grid world size
		float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
		float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;

		// clamp so that if player (worldPos) is outside grid for some reason we don't get a invalid 
		// index to the grid array
		percentX = Mathf.Clamp01 (percentX);
		percentY = Mathf.Clamp01 (percentY);

		// get x, y indexes in the grid array
		int x = Mathf.RoundToInt ((gridSizeX - 1) * percentX);
		int y = Mathf.RoundToInt ((gridSizeY - 1) * percentY);

		return grid[x, y];
		*/
	}

	void OnDrawGizmos() {

		if (mapLoader == null) {

			mapLoader = new MapLoader (new Vector2(20, 20), 0.5f);
			mapData = mapLoader.LoadMap ("A", "endPos", "startPos");
			
			nodeDiameter = mapData.nodeRadius * 2;
			gridWorldSize = mapData.gridWorldSize;
			// #nodes that can fit into the x-space
			gridSizeX = Mathf.RoundToInt (gridWorldSize.x / nodeDiameter);
			gridSizeY = Mathf.RoundToInt (gridWorldSize.y / nodeDiameter);
			CreateGrid ();
		}

		// Draw gridWorldSize
		Gizmos.DrawWireCube (transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

		// Checks if createGrid workes

		if (grid != null) {
			Node playerNode = NodeFromWorldPoint(player.position);
			foreach (Node n in grid) {
				Gizmos.color = (n.walkable) ? Color.white : Color.red;
				if (playerNode == n){
					Gizmos.color = Color.blue;
				}
				Gizmos.DrawCube (n.worldPosition, Vector3.one * (nodeDiameter - .1f));
			}

			/*
			Node nod = grid[4, 3];
			FillNeighbourhood4(nod);
			Gizmos.color = Color.magenta;
			Gizmos.DrawCube (nod.worldPosition, Vector3.one * (nodeDiameter - .1f));
			foreach(Node n in nod.neighbours) {
				Gizmos.color = Color.black;
				Gizmos.DrawCube (n.worldPosition, Vector3.one * (nodeDiameter - .1f));
			}
			*/
		}
	}

	public void FillNeighbourhood16(Node node) {
		FillNeighbourhood8 (node);

		int neighX = node.gridPosX - 2;
		int neighY = node.gridPosY -1 ;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      new Vector3(-2, 0, -1)));
		neighX = node.gridPosX-2;
		neighY = node.gridPosY +1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      new Vector3(1, 0, -2)));

		neighX = node.gridPosX -1;
		neighY = node.gridPosY -2;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                      new Vector3(-1, 0, -2)));       

		neighX = node.gridPosX -1;
		neighY = node.gridPosY +2;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                      new Vector3(-1, 0, 2)));

		neighX = node.gridPosX +1;
		neighY = node.gridPosY -2;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                        new Vector3(1, 0, -2)));

		neighX = node.gridPosX +1;
		neighY = node.gridPosY +2;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                        new Vector3(1, 0, 2)));

		neighX = node.gridPosX +2;
		neighY = node.gridPosY -1;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                        new Vector3(2, 0, -1)));

		neighX = node.gridPosX +2;
		neighY = node.gridPosY +1;
		if (validIndex (neighX, neighY))
			grid [node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint (node.worldPosition + 
			                                                                        new Vector3(2, 0, 1)));

	}

	public void FillNeighbourhood4(Node node) {
		int neighX = node.gridPosX;
		int neighY = node.gridPosY + 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      Vector3.forward));

		neighX = node.gridPosX + 1;
		neighY = node.gridPosY;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      Vector3.right));

		neighX = node.gridPosX;
		neighY = node.gridPosY - 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      Vector3.back));

		neighX = node.gridPosX - 1;
		neighY = node.gridPosY;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                                                      Vector3.left));

	}

	public void FillNeighbourhood8(Node node) {
		int neighX = node.gridPosX - 1;
		int neighY = node.gridPosY - 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.left + Vector3.back));

		neighX = node.gridPosX;
		neighY = node.gridPosY + 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.forward));
		
		neighX = node.gridPosX + 1;
		neighY = node.gridPosY + 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.right + Vector3.forward));
		
		neighX = node.gridPosX - 1;
		neighY = node.gridPosY;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.left));
		
		neighX = node.gridPosX + 1;
		neighY = node.gridPosY;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.right));
		
		neighX = node.gridPosX - 1;
		neighY = node.gridPosY + 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.left + Vector3.forward));
		
		neighX = node.gridPosX;
		neighY = node.gridPosY - 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.back));
		
		neighX = node.gridPosX + 1;
		neighY = node.gridPosY - 1;
		if(validIndex (neighX, neighY))
			grid[node.gridPosX, node.gridPosY].neighbours.Add (NodeFromWorldPoint(node.worldPosition + 
			                                   Vector3.right + Vector3.back));
	}


}
