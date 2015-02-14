using UnityEngine;
using System.Collections;

public class Line {

	public Vector3 point1;
	public Vector3 point2;

	public Line(Vector3 point1, Vector3 point2){
		this.point1 = point1;
		this.point2 = point2;
		}


	
	public bool intersect(Line otherLine){

		Vector2 p = Line.getVector2(point1);
		Vector2 r = Line.getVector2(point2)-p;

		Vector2 q = Line.getVector2 (otherLine.point1);
		Vector2 s = Line.getVector2 (otherLine.point2) - q;


		float rsCross = Line.vec2Cross (r, s);
		float qpr=Line.vec2Cross((q-p),r);
		float qps = Line.vec2Cross ((q - p), s);

		if (rsCross == 0) {
			//Debug.Log("rsCross eq 0");
			//Collinear 
			if(qpr==0){
				float rr=Vector2.Dot(r,r);
				float ss=Vector2.Dot(s,s);
				float pqs=Line.vec2Cross((p-q),s);
				//Overlapping
				if((qpr>=0 && qpr<=rr) || (pqs>=0 && pqs<=ss)){
					return true;
				}
				else{
					return false;
				}
			}
			else{
				//Parallel
				return false;
			}
		} 
		else {
			//Debug.Log("rsCross ne 0");
			float t=qps/rsCross;
			float u=qpr/rsCross;

			//Meet if 0<=t<=1 and 0<=u<=1
			if((t>0 && t<1) && (u>0 && u<1)){
				return true;
			}
			else{
				return false;
			}
				}

		}


	public static float vec2Cross(Vector2 vec1,Vector2 vec2){

		float res = vec1.x * vec2.y - vec1.y * vec2.x;

		return res;
		}

	//To get the Vector2 representation
	public static Vector2 getVector2(Vector3 vec){

		Vector2 res = new Vector2 (vec.x, vec.z);

		return res;
	}

}
