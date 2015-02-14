using UnityEngine;
using System.Collections;

public class TestLine : MonoBehaviour {

	Vector3 point1=new Vector3(0,1,0);
	Vector3 point2 = new Vector3 (-5, 1, 5);
	Vector3 point3=new Vector3(0,1,0);
	Vector3 point4=new Vector3(5,1,10);

	// Use this for initialization
	void Start () {

		Line test1 = new Line (point1, point2);
		Line test2 = new Line (point3, point4);

		Vector2 test = Line.getVector2 (test2.point1);

		Debug.Log ("Vector2:" + test);


		bool temp = test1.intersect (test2);

		Debug.Log ("Intersect? " + temp);

	}


	void OnDrawGizmos() {

		Gizmos.color = Color.black;
		Gizmos.DrawLine (point1,point2);
		Gizmos.DrawLine (point3, point4);

		Gizmos.DrawCube (point1, Vector3.one * (0.5f- .1f));
		Gizmos.DrawCube (point2, Vector3.one * (0.5f- .1f));
		Gizmos.DrawCube (point3, Vector3.one * (0.5f- .1f));
		Gizmos.DrawCube (point4, Vector3.one * (0.5f- .1f));

	}
	

}
