using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class DubinWaypoint : MonoBehaviour {

	public Dubin dubin;
	Line dS = null;

	public float carLength ;
	public float maxWheelAngle;
	public bool carMadeIt = false;
	public float vel ;
	float minRadius = 0f;

	List<Vector3> path = null;

	void Start () {
		float maxWheelRad = maxWheelAngle * (Mathf.PI / 180.0f);
		//Debug.Log ("MaxWheelRad:" + maxWheelRad);
		minRadius = Mathf.Abs (carLength / (Mathf.Tan (maxWheelRad) ));
		//Debug.Log ("minRadius:" + minRadius);
		dubin = gameObject.AddComponent<Dubin> ();

		List<Vector3> prePath = new List<Vector3> ();
		prePath.Add (Vector3.zero);
		prePath.Add (new Vector3 (10, 1, 0));

		path = new List<Vector3> ();
		Vector3 start = prePath [0];
		Vector3 goal = prePath [1];

		Transform tmp = new GameObject ().transform;
		Quaternion theta = Quaternion.LookRotation (start - goal);
		tmp.rotation = Quaternion.RotateTowards (tmp.rotation, theta, 9000);

		// Two different goal angles
		//Line S = dubin.MinTrajectory (start, goal, transform.rotation, tmp.rotation, minRadius, minRadius);
		path.Add (prePath [0]);
		//path.Add (S.point1);
		//path.Add (S.point2);
		path.Add (prePath [1]);
		path.Add (new Vector3 (10, 1, -10));
		path.Add (new Vector3 (-20, 1, 15));
		path.Add (new Vector3 (-20, 1, 13));
		path.Add (new Vector3 (-20, 1, 30));
		path.Add (Vector3.zero);
		StartCoroutine ("Move");

		
	//	Destroy (tmp.gameObject);
	}

	int index = 1;
	Vector3 current = Vector3.zero;
	public float goalInterval;
	IEnumerator Move() {
		current = path [index];


		DubinRet S = dubin.MinTrajectory (transform.position, current, transform.rotation, 
		                              transform.rotation, minRadius, minRadius);

		dS = new Line(S.waypoints[0], S.waypoints[1]);

        bool followS = true;
		int q = 2;
        while (true) {
			if(carMadeIt) {
				if(Vector3.Distance (transform.position, path[index]) < goalInterval){
					index++;
					
					if(index >= path.Count)
						yield break;

					followS = true;

					current = path[index];
					S = dubin.MinTrajectory (transform.position, current, transform.rotation, 
					                                       transform.rotation, minRadius, minRadius);
					dS = new Line(S.waypoints[0], S.waypoints[1]);	
					q = 2;
                }
                
			}

			if(followS) {

				if(q == 2){
					current = S.waypoints[0];
					if(Vector3.Distance (transform.position, current) < goalInterval)
						q--;
				}
				if(q == 1)
					current = S.waypoints[1];
				if(Vector3.Distance (transform.position, current) < goalInterval)
						q--;
				if(q == 0) {
					followS = false;
					current = path[index];
				}
			}

			float wheelAngleRad=maxWheelAngle*(Mathf.PI/180);
			float dTheta=(vel/carLength)*Mathf.Tan(wheelAngleRad);
			Quaternion theta = Quaternion.LookRotation (current - transform.position);
			
			if(transform.rotation!=theta){
				transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, dTheta);
			}
			
			Vector3 curDir=transform.eulerAngles;
			Vector3 newPos=transform.position;
			float angleRad=curDir.y*(Mathf.PI/180);
			newPos.x=newPos.x+(vel*Mathf.Sin(angleRad)*Time.deltaTime);
			newPos.z=newPos.z+(vel*Mathf.Cos(angleRad)*Time.deltaTime);
			transform.position=newPos;
			
			if(Vector3.Distance (current, transform.position) < goalInterval)
				carMadeIt = true;
			
			yield return null;
		}
	}



	void OnDrawGizmos() {
		if (dubin != null) {

			
			Gizmos.color = Color.blue;
			if (dubin.tangents != null) {
				foreach(Line line in dubin.tangents) {
					Gizmos.DrawLine (line.point1, line.point2);
				}
			}
			
            Gizmos.color = Color.black;
            if (dubin.proxCircles != null) {
                foreach (Circle circle in dubin.proxCircles) {
					Gizmos.DrawWireSphere (circle.pos, minRadius);
				}
			}

			Gizmos.color = Color.blue;
			if (dubin.tangents != null) {
				foreach (Line line in dubin.tangents) {
					Gizmos.DrawLine (line.point1, line.point2);
				}
			}

			Gizmos.color = Color.green;
			if(dS != null) {
				Gizmos.DrawLine (dS.point1, dS.point2);
			}

			Gizmos.color = Color.red;
			for(int i = index; i < path.Count; i++) {
				Gizmos.DrawCube (path[i], Vector3.one * 0.2f);
			}

			Gizmos.color = Color.magenta;
			if(current != Vector3.zero){
				Gizmos.DrawCube (current, Vector3.one * 0.5f);
			}


        }
    }	
}
