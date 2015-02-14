using UnityEngine;
using System.Collections;

public class MapData {
	public bool[,] walkable;
	public Vector2 start;
	public Vector2 end;
	public Vector2 gridWorldSize;
	public float nodeRadius;
}