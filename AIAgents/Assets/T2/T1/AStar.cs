using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class AStar : MonoBehaviour{
	Node GetNode(Node currentNode, int dir) {
		List<Node> directions = currentNode.neighbours;
		return directions [dir];
	}

	public List<Node> AStarSearch(Node startNode, Node targetNode) {
		
		print ("start node: " + startNode.gridPosX + ", " + startNode.gridPosY);
		print ("target node: " + targetNode.gridPosX + ", " + targetNode.gridPosY);
		print (targetNode.worldPosition);
		if (startNode == targetNode)
			return new List<Node>();
		
		PriorityQueue<Node, float> frontier = new PriorityQueue<Node, float> ();
		Dictionary<Node, Node> cameFrom = new Dictionary<Node, Node> ();
		Dictionary<Node, float> costSoFar = new Dictionary<Node, float> ();

		frontier.Enqueue(startNode, 0f);
		cameFrom [startNode] = null;
		costSoFar [startNode] = 0;

		Node currentNode;
		while (frontier.Count() != 0) {
			currentNode = frontier.Dequeue ();

			if(currentNode == targetNode) {
				return ConstructPath (startNode, targetNode, cameFrom);
			}

			foreach(Node node in currentNode.neighbours){
				float newCost = costSoFar[currentNode] + GetCost (currentNode, node);
				if (!costSoFar.ContainsKey (node) || newCost < costSoFar[node]) {
					if(node.walkable) {
						costSoFar[node] = newCost;
						node.cost=newCost;
						if(node==targetNode){
							Debug.Log("TargetNode:"+newCost);
						}
						float priority = newCost + Heuristic (targetNode.worldPosition, node.worldPosition);
						frontier.Enqueue (node, priority);
						cameFrom[node] = currentNode;
					}
				}
			}
		}

		return new List<Node> ();
	}

	public List<Node> BFS(Node startNode, Node targetNode) {
		Queue<Node> frontier = new Queue<Node> ();
		frontier.Enqueue (startNode);
		Dictionary<Node, Node> cameFrom = new Dictionary<Node, Node> ();
		cameFrom [startNode] = null;
		
		while (frontier.Count != 0) {
			Node currentNode = frontier.Dequeue();
			
			if(currentNode == targetNode) {
				return ConstructPath (startNode, targetNode, cameFrom);
			}
			
			foreach(Node node in currentNode.neighbours) {
				if(!cameFrom.ContainsKey (node)) {
					if(node.walkable) {
						frontier.Enqueue(node);
						cameFrom[node] = currentNode;
					}
				}
			}
		}
		return new List<Node> ();
	}

	float Heuristic(Vector3 A, Vector3 B) {
		return Mathf.Abs (A.x - B.x) + Mathf.Abs (A.y - B.y);
	}

	List<Node> ConstructPath(Node start, Node target, Dictionary<Node, Node> cameFrom) {
		Node currentNode = target;
		List<Node> path = new List<Node>();
		while (currentNode != start) {
			path.Add (currentNode);
			currentNode = cameFrom[currentNode];
		}
		path.Reverse ();

		return path;
	}

	public float GetCost(Node from, Node to) {
		// TODO: Perhaps check if neighbours else return infinity

		float straightCost = 1f;//10f;
		float diagonalCost = Mathf.Sqrt(2);
		float skipCost = 5f;
		/*
		if (from.gridPosX == to.gridPosX && from.gridPosY + 2 == to.gridPosY) // up
			return skipCost;
		if (from.gridPosX + 2 == to.gridPosX && from.gridPosY == to.gridPosY) // right
			return skipCost;
		if (from.gridPosX == to.gridPosX && from.gridPosY - 2 == to.gridPosY) // down
			return skipCost;
		if (from.gridPosX - 2 == to.gridPosX && from.gridPosY == to.gridPosY) // left
			return skipCost;*/

		if (from.gridPosX == to.gridPosX && from.gridPosY + 1 == to.gridPosY) // up
			return straightCost;
		if (from.gridPosX + 1 == to.gridPosX && from.gridPosY == to.gridPosY) // right
			return straightCost;
		if (from.gridPosX == to.gridPosX && from.gridPosY - 1 == to.gridPosY) // down
			return straightCost;
		if (from.gridPosX - 1 == to.gridPosX && from.gridPosY == to.gridPosY) // left
			return straightCost;

		return diagonalCost;

	}
}
