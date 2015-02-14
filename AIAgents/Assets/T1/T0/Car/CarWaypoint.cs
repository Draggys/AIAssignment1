using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CarWaypoint : MonoBehaviour {

	public List<Vector3> path;
	public int model;
	public float vel;
	GameObject rightFrontWheel;
	GameObject leftFrontWheel;

	void Start () {
		model = 0;
		vel = 30f;
		Vector3 pos1 = new Vector3 (0, 0, 30);
		Vector3 pos2 = new Vector3 (30, 0, 30);
		Vector3 pos3 = new Vector3 (30, 0, 0);
		path = new List<Vector3> ();
		path.Add (pos1);
		path.Add (pos3);
		path.Add (pos2);

		rightFrontWheel = GameObject.Find ("RightTopWheel");
		leftFrontWheel = GameObject.Find ("LeftTopWheel");

		StartCoroutine ("Move", model);
	}
	public int index = 1;
	IEnumerator Move(int model) {
		Vector3 	current = path[index];
		while (true) {
			if(transform.position == current) {
				index++;
				if(index >= path.Count) {
					yield break;
				}
				current = path[index];
			}
			// Kinematic car model
			else if(model == 0) {

				Quaternion phi = Quaternion.LookRotation (current - rightFrontWheel.transform.position);
				Quaternion lphi = Quaternion.LookRotation (current - leftFrontWheel.transform.position);
				/*rightFrontWheel.transform.rotation = Quaternion.RotateTowards(rightFrontWheel.transform.rotation,
				                                                              phi, 20 * Time.deltaTime);
				  */  


				rightFrontWheel.transform.eulerAngles = phi.eulerAngles;
				leftFrontWheel.transform.eulerAngles = lphi.eulerAngles;
				transform.eulerAngles = new Vector3(0, (vel / 1) * Mathf.Tan (phi.y), 0);
				rightFrontWheel.transform.eulerAngles = new Vector3(0, 90, 0);

				Vector3 dir = Vector3.Normalize(current - transform.position);
				dir.x = dir.x + vel * Mathf.Cos (phi.y) * Time.deltaTime * 0.01f;
				dir.z = dir.z + vel * Mathf.Sin (phi.y) * Time.deltaTime * 0.01f;
				transform.position = transform.position + dir;
				/*
				float phi2 = Vector3.Angle (transform.right, rightFrontWheel.transform.forward);

				//transform.eulerAngles = transform.eulerAngles + new Vector3(0, 30 * Mathf.Tan (phi), 0);

*/
				/*
				float theta =  Mathf.Tan (phi.y);
				print ("theta = " + theta);
				transform.rotation = Quaternion.AngleAxis (theta, Vector3.up);

				//float theta = (vel / 2) * Mathf.Tan (phi.y);
				//transform.eulerAngles = new Vector3(0, theta, 0);
*/


				yield return null;
			}
			else {
				yield break;
			}
		}
	}

	public void OnDrawGizmos() {
		if (path != null) {
			for(int i = 0; i < path.Count; i++) {
				Gizmos.color = Color.white;
				Gizmos.DrawCube (path[i], Vector3.one);
			}

			Gizmos.color = Color.red;
			Gizmos.DrawLine (transform.position, transform.right);
			Gizmos.color = Color.blue;
			Gizmos.DrawLine (rightFrontWheel.transform.position, rightFrontWheel.transform.forward + rightFrontWheel.transform.position);

		
		}
	}
}
