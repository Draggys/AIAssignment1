using UnityEngine;
using System.Collections;
using System;

public class MapLoader{
	private string prefix = Application.dataPath+"/Data/Discrete/";
	private string postfix = ".txt";

	public string mapName;
	public string startName;
	public string endName;

	public int maxX;
	public int maxY;

	public Vector2 gridWorldSize;
	public float nodeRadius;

	public MapLoader(Vector2 gridWorldSize, float nodeRadius) {
		this.gridWorldSize = gridWorldSize;
		this.nodeRadius = nodeRadius;

		maxX = (int) Math.Round (gridWorldSize.x);
		maxY = (int) Math.Round (gridWorldSize.y);
	}

	public MapData LoadMap(string mapName, string startName, string endName)
	{
		mapName = prefix + mapName + postfix;
		startName = prefix + startName + postfix;
		endName = prefix + endName + postfix;

		MapData mapData = new MapData ();
		mapData.walkable = new bool[(int) Math.Round (gridWorldSize.x), (int) Math.Round (gridWorldSize.y)];

		if (!System.IO.File.Exists (mapName)) {
			Debug.Log("File Not Found");
		}

		// Read map
		int x = 0;
		string line;
		System.IO.StreamReader file = new System.IO.StreamReader (mapName);
		while ((line = file.ReadLine ()) != null) {
			string[] walkable = line.Split (' ');
			for(int y = 0; y < walkable.Length; y++) {
				if(walkable[y].Equals ("1"))
					mapData.walkable[x, y] = false;
				else
					mapData.walkable[x, y] = true;
			}
			x++;
		}

		file.Close ();

		// read start and end pos
		string start = System.IO.File.ReadAllText (startName);
		string[] startList = start.Split (' ');
		mapData.start = new Vector2(float.Parse (startList[0]) - 1, float.Parse (startList[1]) - 1);

		string end = System.IO.File.ReadAllText (endName);
		string[] endList = end.Split (' ');
		mapData.end = new Vector2 (float.Parse (endList [0]) - 1, float.Parse (endList [1]) - 1);

		mapData.gridWorldSize = gridWorldSize;
		mapData.nodeRadius = nodeRadius;
		return mapData;
	}
}