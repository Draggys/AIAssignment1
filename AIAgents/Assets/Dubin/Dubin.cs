using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Circle {
	public Vector3 pos;
	public string type;

	public Circle(Vector3 pos, string type) {
		this.pos = pos;
		this.type = type;
	}
}

public class CSC {
	public float cost;
	public Line tangent;

	public CSC(Line tangent) {
		this.tangent = tangent;
		cost = 0;
	}
}

public class DubinRet {
	public float cost;
	public List<Vector3> waypoints;

	public DubinRet(Vector3 p1, Vector3 p2, float cost) {
		this.cost = cost;
		waypoints = new List<Vector3> ();
		waypoints.Add (p1);
		waypoints.Add (p2);
	}
}

public class CCC {
	public List<Vector3> arcPos;
	public float cost;

	public CCC(Vector3 pt1, Vector3 pt2) {
		arcPos = new List<Vector3> ();
		arcPos.Add (pt1);
		arcPos.Add (pt2);
		cost = 0;
	}
}

public class Dubin : MonoBehaviour {
	public List<CSC> CSC = null;
	public List<CCC> CCC = null;

	// Debug
	public Circle[] proxCircles = null;
	public List<Line> tangents = null;

	public DubinRet MinTrajectory(Vector3 start, Vector3 goal, Quaternion startAngle, Quaternion goalAngle,
	                          float r1, float r2) {
		tangents = new List<Line> ();

		CSC csc = CSCMinTrajectoryInner(start, goal, startAngle, goalAngle, r1, r2);
		CCC ccc= CCCMinTrajectory (start, goal, startAngle, goalAngle, r1, r2);

	//	print ("csc cost " + csc.cost);
	//	print ("ccc cost " + ccc.cost);

		if (float.IsNaN (ccc.cost) || csc.cost < ccc.cost) {
			return new DubinRet(csc.tangent.point1, csc.tangent.point2, csc.cost);
		} 
		else {
	//		print ("CCC");
			return new DubinRet(ccc.arcPos[0], ccc.arcPos[1], ccc.cost);
		}

		/*
        Line SI = CSCMinTrajectoryInner(start, goal, startAngle, goalAngle, r1, r2);
		//Line SO = CCCMinTrajectoryOuter(start, goal, startAngle, goalAngle, r1, r2);
		Line SO = null;
		//return SI;

		if (SI == null && SO != null)
			return SO;
		else if (SO == null && SI != null)
			return SI;
		else if (Vector3.Distance (SI.point2, SI.point1) < Vector3.Distance (SO.point2, SO.point1))
			return SI;
		else
			return SO;
		*/
	}

	public CCC CCCMinTrajectory(Vector3 start, Vector3 goal, Quaternion startAngle, Quaternion goalAngle,
	                             float r1, float r2) {
		CCC = new List<CCC> ();
		float rMin;
		if (r1 < r2)
			rMin = r1;
		else
			rMin = r2;

		Vector3 p1 = start;
		Vector3 p2 = goal;
		float p1p3 = 2 * rMin;
		float p2p3 = 2 * rMin;
		float p1p2 = Mathf.Sqrt ((goal.x - start.x) * (goal.x - start.x) +
						(goal.z - start.z) * (goal.z - start.z));
		Vector3 V1 = p2 - p1;
		float theta = Mathf.Acos (p1p2 / (4 * rMin));

		// LRL
		Vector3 p3 = new Vector3 (start.x + 2 * rMin * Mathf.Cos (theta), 1, start.z + 2 * rMin * Mathf.Sin(theta));
		Vector3 V2 = p1 - p3;
		V2 = V2.normalized * rMin;
		Vector3 pt1 = p3 + V2;
		V2 = p2 - p3;
		V2 = V2.normalized * rMin;
		Vector3 pt2 = p3 + V2;

		CCC ccc = new CCC (pt1, pt2);
		ccc.cost += ArcLength (p1, start, pt1, "L", r1);
		ccc.cost += ArcLength (p3, pt1, pt2, "R", r1);
		ccc.cost += ArcLength (p2, pt2, goal, "L", r1);
		CCC.Add (ccc);

		// RLR
		p3 = new Vector3 (start.x - 2 * rMin * Mathf.Cos (theta), 1, start.z - 2 * rMin * Mathf.Sin(theta));
		V2 = p1 - p3;
		V2 = V2.normalized * rMin;
		pt1 = p3 + V2;
		V2 = p2 - p3;
		V2 = V2.normalized * rMin;
		pt2 = p3 + V2;

		ccc = new CCC (pt1, pt2);
		ccc.cost += ArcLength (p1, start, pt1, "R", r1);
		ccc.cost += ArcLength (p3, pt1, pt2, "L", r1);
		ccc.cost += ArcLength (p2, pt2, goal, "R", r1);
		CCC.Add (ccc);

		// min
		int best = 0;
		float bestCost = -1;
		for (int i = 0; i < CCC.Count; i++) {
			//	print (CSC[i].cost);
			if(bestCost == -1 || CCC[i].cost < bestCost) {
				best = i;
				bestCost = CCC[i].cost;
			}
		}

		return CCC[best];
	}

	public CSC CSCMinTrajectoryInner(Vector3 start, Vector3 goal, Quaternion startAngle, Quaternion goalAngle,
	                   float r1, float r2) {

		CSC = new List<CSC> ();
		CSCTrajectories(start, goal, startAngle, goalAngle, r1, r2, "inner");

		int best = 0;
		float bestCost = -1;
		for (int i = 0; i < CSC.Count; i++) {
		//	print (CSC[i].cost);
			if(bestCost == -1 || CSC[i].cost < bestCost) {
				best = i;
				bestCost = CSC[i].cost;
			}
		}

		return CSC [best];
	}

	void CSCTrajectories(Vector3 start, Vector3 goal, Quaternion startAngle, Quaternion goalAngle,
	                    float r1, float r2, string type) {
		proxCircles = GetProximityCircles (start, goal, startAngle, goalAngle, r1, r2);
		List<Line> possibleTangents;

		// RSR
		//if(type == "inner")
		//	possibleTangents = CalculateTangents (proxCircles [0].pos, proxCircles [2].pos, r1, r2, "inner");
		//else
			possibleTangents = CalculateTangents (proxCircles [0].pos, proxCircles [2].pos, r1, r2, "outer","R");
		foreach (Line tangent in possibleTangents) {
			float angle = Vector3.Angle (transform.position, tangent.point1);
			if(angle < 90){
				tangents.Add (tangent);

				CSC csc = new CSC(tangent);
				Vector3 p1 = proxCircles[0].pos;
				Vector3 p2 = start;
				Vector3 p3 = tangent.point1;
				csc.cost += ArcLength (p1, p2, p3, "R", r1);
				csc.cost += Vector3.Distance (tangent.point1, tangent.point2);
				p1 = proxCircles[1].pos;
				p2 = tangent.point2;
				p3 = goal;
				csc.cost += ArcLength (p1, p2, p3, "R", r1);
				CSC.Add (csc);
			}
		}

		//RSL
		//if(type == "inner")
			possibleTangents = CalculateTangents (proxCircles [0].pos, proxCircles [3].pos, r1, r2, "inner","R");
		//else
		//	possibleTangents = CalculateTangents (proxCircles [0].pos, proxCircles [3].pos, r1, r2, "outer");
		foreach (Line tangent in possibleTangents) {
			float angle = Vector3.Angle (transform.position, tangent.point1);
			if(angle < 90){
				tangents.Add (tangent);

				CSC csc = new CSC(tangent);
				Vector3 p1 = proxCircles[0].pos;
				Vector3 p2 = start;
				Vector3 p3 = tangent.point1;
				csc.cost += ArcLength (p1, p2, p3, "R", r1);
				csc.cost += Vector3.Distance (tangent.point1, tangent.point2);
				p1 = proxCircles[1].pos;
				p2 = tangent.point2;
				p3 = goal;
				csc.cost += ArcLength (p1, p2, p3, "L", r1);
				CSC.Add (csc);
			}
		}
		
		// LSR
		//if(type == "inner")
			possibleTangents = CalculateTangents (proxCircles [1].pos, proxCircles [2].pos, r1, r2, "inner","L");
		//else
		//	possibleTangents = CalculateTangents (proxCircles [1].pos, proxCircles [2].pos, r1, r2, "outer");
		foreach (Line tangent in possibleTangents) {
			float angle = Vector3.Angle (transform.position, tangent.point1);
			if(angle < 90){
				tangents.Add (tangent);

				
				CSC csc = new CSC(tangent);
				Vector3 p1 = proxCircles[0].pos;
				Vector3 p2 = start;
				Vector3 p3 = tangent.point1;
				csc.cost += ArcLength (p1, p2, p3, "L", r1);
				csc.cost += Vector3.Distance (tangent.point1, tangent.point2);
				p1 = proxCircles[1].pos;
				p2 = tangent.point2;
				p3 = goal;
				csc.cost += ArcLength (p1, p2, p3, "R", r1);
				CSC.Add (csc);
			}
		}

		// LSL
		//if(type == "inner")
		//	possibleTangents = CalculateTangents (proxCircles [1].pos, proxCircles [3].pos, r1, r2, "inner");
		//else
			possibleTangents = CalculateTangents (proxCircles [1].pos, proxCircles [3].pos, r1, r2, "outer","L");
		foreach (Line tangent in possibleTangents) {
			float angle = Vector3.Angle (transform.position, tangent.point1);
			if(angle < 90){
				tangents.Add (tangent);

				
				CSC csc = new CSC(tangent);
				Vector3 p1 = proxCircles[0].pos;
				Vector3 p2 = start;
				Vector3 p3 = tangent.point1;
				csc.cost += ArcLength (p1, p2, p3, "L", r1);
				csc.cost += Vector3.Distance (tangent.point1, tangent.point2);
				p1 = proxCircles[1].pos;
				p2 = tangent.point2;
				p3 = goal;
				csc.cost += ArcLength (p1, p2, p3, "L", r1);
				CSC.Add (csc);
			}
		}

	}

	Circle[] GetProximityCircles(Vector3 startPos, Vector3 goalPos, Quaternion startAngle, Quaternion goalAngle, 
	                              float r1, float r2) {
		Transform current = new GameObject().transform;
		current.position = startPos;
		current.rotation = startAngle;
		Transform goal = new GameObject().transform;
		goal.position = goalPos;
		goal.rotation = goalAngle;
		//Debug.Log ("current.euler " + current.eulerAngles);
		float rotAngle = (current.eulerAngles.y-180-90);
		rotAngle = rotAngle * (Mathf.PI / 180.0f);
		float zPos = Mathf.Cos (rotAngle);
		//Debug.Log ("XPos:" + xPos);
		float xPos = Mathf.Sin (rotAngle);
		//Debug.Log ("zPos:" + zPos);
		Vector3 direction = new Vector3 (xPos, 0, zPos);
		float rotAngle2 = (goal.eulerAngles.y-180-90);
		//Debug.Log ("Direction:" + direction);
		rotAngle2 = rotAngle2 * (Mathf.PI / 180.0f);
		float zPos2 = Mathf.Cos (rotAngle2);
		float xPos2 = Mathf.Sin (rotAngle2);
		Vector3 direction2 = new Vector3 (xPos2, 0, zPos2);
		//Debug.Log ("Direction2:" + direction2);
		//Debug.Log ("GoalAngle:" + goalAngle.eulerAngles);
		Circle[] ret = {new Circle(current.position + direction * r1, "R"),
			new Circle(current.position - direction * r1, "L"),
			new Circle(goal.position + direction2 * r2, "R"), 
			new Circle(goal.position - direction2 * r2, "L")};
			
		Destroy (current.gameObject);
		Destroy (goal.gameObject);
		return ret;
	}

	List<Line> CalculateTangents(Vector3 p1, Vector3 p2, float r1, float r2, string type, string Dir){
		Vector3 V1 = p2 - p1;
		float D = Vector3.Magnitude (V1);

		Vector3 p3 = new Vector3((p1.x + p2.x)/2, 1, (p1.z + p2.z) / 2);
		float r3 = D / 2;

		float r4 = r1 + r2;
		if (type == "outer") {
			r4 = r1 - r2; // Assumption: r1 >= r2
		}

		/*
		float theta = vel / L + Mathf.Atan2 (V1.z, V1.x);
		Vector3 pt = new Vector3 (p1.x + r4 * Mathf.Cos (theta), 1,
		                          p1.z + r4 * Mathf.Sin (theta));
		                          */

		List<Vector3> pts = IntersectionPoints (p1, p3, r4, r3);
		if (pts == null) {
		//	print ("No intersection was found");
			return new List<Line> ();
		}


		List<Line> ret = new List<Line> ();

		if (pts.Count == 1) {

			Vector3[] tangent1 = CalculateTangentHelper (pts [0], p1, p2, r1, Dir);
			ret.Add (new Line (tangent1 [0], tangent1 [1]));

			return ret;

				}


		Vector3 interSec1 = Vector3.Normalize(p1 - pts [0]);
		Vector3 interSec2 = Vector3.Normalize(p1 - pts [1]);
		Vector3 v1Norm = Vector3.Normalize (V1);
		float ang1 = Mathf.Acos (Vector3.Dot (interSec1, v1Norm));
		float ang2 = Mathf.Acos (Vector3.Dot (interSec2, v1Norm));

		//If the circle direction is left we want to use the "low" intersection,
		//i.e. the intersection that have a angle >180 degrees from V1
		int intersectionToBeUsed = -1;
		if (Dir == "L") {

			if(ang1>180){
				intersectionToBeUsed=0;
			}
			else{
				intersectionToBeUsed=1;
			}
				} 
		else {
			if(ang1<180){
				intersectionToBeUsed=0;
			}
			else{
				intersectionToBeUsed=1;
			}
				}


		Vector3[] tangent = CalculateTangentHelper (pts [intersectionToBeUsed], p1, p2, r1, Dir);
		ret.Add (new Line (tangent [0], tangent [1]));
		/*if (pts.Count == 2) {
			tangent = CalculateTangentHelper (pts [1], p1, p2, r1);
			ret.Add (new Line (tangent [0], tangent [1]));
		}*/

		// DEBUGGING
		/*
		circles = new List<KeyValuePair<Vector3, float> > ();
		circles.Add (new KeyValuePair<Vector3, float> (p1, r4));
		circles.Add (new KeyValuePair<Vector3, float> (p3, r3));
		circles.Add (new KeyValuePair<Vector3, float> (p2, r2));
		circles.Add (new KeyValuePair<Vector3, float> (p1, r1));
		*/
		return ret;
	}
	
	Vector3[] CalculateTangentHelper(Vector3 pt, Vector3 p1, Vector3 p2, float r1, string Dir) {
		//Outer tangent
		if (pt == p1) {
			Vector3 V1=p2-p1;
			//This will give us a vector orthogonal to V1
			Vector3 normal=Vector3.Normalize(Vector3.Cross(Vector3.up,V1));

		//	Debug.Log("Normal:"+normal);

			Vector3 point1=p1+normal*r1;
			Vector3 point2=p1-normal*r1;

			Vector3 point1P2=p2+normal*r1;
			Vector3 point2P2=p2-normal*r1;

			Vector3 interSec1 = Vector3.Normalize(p1 - point1);
			Vector3 interSec2 = Vector3.Normalize(p1 - point2);
			Vector3 v1Norm = Vector3.Normalize (V1);
			float ang1 = Mathf.Acos (Vector3.Dot (interSec1, v1Norm));
			float ang2 = Mathf.Acos (Vector3.Dot (interSec2, v1Norm));
			
			//If the circle direction is left we want to use the "low" intersection,
			//i.e. the intersection that have a angle >180 degrees from V1

			if (Dir == "L") {
				
				if(ang1>180){
					Vector3[] retVal = {point1, point1P2};
					return retVal;
				}
				else{
					Vector3[] retVal = {point2, point2P2};
					return retVal;
				}
			} 
			else {
				if(ang1<180){
					Vector3[] retVal = {point1, point1P2};
					return retVal;
				}
				else{
					Vector3[] retVal = {point2, point2P2};
					return retVal;
				}
			}


				} else {
						Vector3 V2 = pt - p1;
						Vector3 V3 = Vector3.Normalize (V2) * r1;
						Vector3 pit1 = p1 + V3;
		
						Vector3 V4 = (p2 - pt);
						Vector3 pit2 = new Vector3 (pit1.x + V4.x, 1, pit1.z + V4.z);
		
						Vector3[] ret = {pit1, pit2};
						return ret;
				}
	}

	List<Vector3> IntersectionPoints(Vector3 p1, Vector3 p2, float r1, float r2) {
		float dx = p1.x - p2.x;
		float dy = p1.z - p2.z;
		float dist = Mathf.Sqrt (dx * dx + dy * dy);

		if (dist > r1 + r2) {
			return null;
		}
		else if (dist < Mathf.Abs (r1 - r2)) {
			return null;
		}
		else if ((dist == 0) && (r1 == r2)) {
			return null;
		}
		else {
			float a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
			float h = Mathf.Sqrt (r1 * r1 - a * a);

			Vector3 center = new Vector3(p1.x + a * (p2.x - p1.x) / dist, 1, 
			                             p1.z + a * (p2.z - p1.z) / dist);

			Vector3 i1 = new Vector3(center.x - h * (p2.z - p1.z) / dist, 1, 
			                         center.z + h * (p2.x - p1.x) / dist);

			Vector3 i2 = new Vector3(center.x + h * (p2.z - p1.z) / dist, 1, 
			                         center.z - h * (p2.x - p1.x) / dist);

			List<Vector3> ret = new List<Vector3> ();
			if(dist == r1 + r2){
				ret.Add (i1);
				return ret;
			}
			else {
				ret.Add (i1);
				ret.Add (i2);
				return ret;
			}
		}
	}

	// p1 is circle origo
	// p2 is start on circle
	// p3 is goal on circle
	// d is the type of circle
	// r1 is the radius of the circle
	float ArcLength(Vector3 p1, Vector3 p2, Vector3 p3, string d, float r1) {
		Vector3 V1 = p2 - p1;
		Vector3 V2 = p3 - p1;
		
		float theta = Mathf.Atan2 (V2.z, V2.x) - Mathf.Atan2 (V1.z, V1.x);
		if (theta < 0 && d == "L") {
			theta = theta + 2 * Mathf.PI;
		} 
		else if (theta > 0 && d == "R") {
			theta = theta - 2 * Mathf.PI;
		}

		return Mathf.Abs (theta * r1);
	}

	void OnDrawGizmos() {
		

        /*
		Gizmos.color = Color.black;
		if (circles != null) {
			foreach (KeyValuePair<Vector3, float> p in circles) {
				Gizmos.DrawWireSphere(p.Key, p.Value);
			}
		}
		Gizmos.color = Color.blue;
		if (tangents != null) {
			foreach(Line line in tangents) {
				Gizmos.DrawLine (line.point1, line.point2);
            }
        }

		Gizmos.color = Color.white;
		if (proxCircles != null) {
			foreach(Circle circle in proxCircles) {
				Gizmos.DrawWireSphere(circle.pos, minRadius);
			}
		}

		Gizmos.color = Color.red;
		Gizmos.DrawLine (start, startDir);
		Gizmos.DrawLine (goal, goalDir);

		Gizmos.color = Color.green;
		if(winningTangent != null)
			Gizmos.DrawLine (winningTangent.point1, winningTangent.point2);
			*/
	}

}
