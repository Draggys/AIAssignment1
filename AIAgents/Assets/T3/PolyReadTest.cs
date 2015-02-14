using UnityEngine;
using System.Collections;

public class PolyReadTest : MonoBehaviour {

	PolyMapLoader map;
	PolyGraphCreator createdGraph;

	// Use this for initialization
	void Start () {
		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");	

		Debug.Log ("Lines size:" + map.polyData.lines.Count);
		Debug.Log ("Vertices size:" + map.polyData.nodes.Count);
		


		//Debug.Log ("Created Graph size:" + createdGraph.possibleLines.Count);

		/*
		 * Debugging
		map.polyData.printNodes ();
		map.polyData.printStart ();
		map.polyData.printEnd ();
		map.polyData.printButtons ();
		*/
	}

	void OnDrawGizmos() {

		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");
		
		if (map.polyData.nodes != null) {
			foreach(Vector3 node in map.polyData.nodes) {
				Gizmos.color = Color.blue;
				Gizmos.DrawCube (node, Vector3.one);
			}

			foreach(Line line in map.polyData.lines){
				Gizmos.color=Color.black;

				Gizmos.DrawLine(line.point1,line.point2);
			}

		}
		Gizmos.color = Color.green;
		Gizmos.DrawCube (map.polyData.start, Vector3.one);
		
		Gizmos.color = Color.red;
		Gizmos.DrawCube (map.polyData.end, Vector3.one);

		Gizmos.color = Color.black;
		Gizmos.DrawLine (new Vector3 (0, 1, 0), new Vector3 (100, 1, 0));
		Gizmos.DrawLine (new Vector3 (100, 1, 0),new Vector3(100,1,90));
		Gizmos.DrawLine (new Vector3(100,1,90), new Vector3 (0, 1, 90));
		Gizmos.DrawLine (new Vector3(0,1,90), new Vector3 (0, 1, 0));

	}	
}
	