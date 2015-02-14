using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Waypoints : MonoBehaviour {
	
	List<Vector3> path;
	private List<Vector3> velAtPoint;

	public float goalInterval;


	//For Kinematic Point Model
	public float kinematic_vel;
	//For Dynamic Point Model
	public float accMax;
	public float dynFx ;
	public float dynFy ;
	public float accX;
	public float accY;
	public float dynM ;
	//For Differential Drive
	public float vel ;
	public float angleVel;
	public bool isRunning = false;

	//For Kinematic Car Model
	public float carLength;
	public float maxWheelAngle;
	public bool carMadeIt = false;
	//For Dynamic Car Model
	//public float dynVel;
	public float dynF;
	public float dynMass;

	
	// models from 1 - 3
	public int model ;
	
	// Use this for initialization
	void Start () {
		Vector3 pos1 = new Vector3 (0, 1, 0);
		Vector3 pos2 = new Vector3 (0, 1, 10);
		Vector3 pos3 = new Vector3 (-25, 1, 10);
		Vector3 pos4 = new Vector3 (10, 1, -9);
		Vector3 pos5 = new Vector3 (-15, 1, 15);
		path = new List<Vector3> ();
		path.Add (pos1);
		path.Add (pos2);
		path.Add (pos3);
		path.Add (pos4);
		path.Add (pos5);
		path.Add (new Vector3 (-17, 1, -17));

		for (int i=0; i<5; i++) {
						path.Add (pos1);
						path.Add (pos2);
						path.Add (pos3);
						path.Add (pos4);
						path.Add (pos5);
						path.Add (new Vector3 (-17, 1, -17));
				}


		velAtPoint=new List<Vector3>();


		
		//StartCoroutine ("Move", model);
	}
	
	void Update(){
		
		if (Input.GetMouseButtonDown (0) && !isRunning) {
			isRunning=true;
			Debug.Log("Mouse Down");
			StartCoroutine ("Move", model);
			
		}
		
	}
	
	IEnumerator Move(int model) {
		int index = 0;
		float velX = 0;
		float velY = 0;
		float dynVel = 0;
		Vector3 dynPVel = new Vector3 (0,0,0);

		//test
		Vector3 goalVel = new Vector3 (1, 0, 1);

		Vector3 current = path[index];
		while (true) {
			float distance=Vector3.Distance(transform.position,current);
			if(distance<=goalInterval || carMadeIt) {
				index++;
				carMadeIt=false;
				Debug.Log("Arrived at position");
				//Debug.Log ("DynVel:"+dynVel);
				//Debug.Log("DynPVel:"+dynPVel);
				if(index >= path.Count) {
					isRunning=false;
					yield break;
				}
				velX=0;
				velY=0;
				//dynVel=0;
				current = path[index];
				Debug.Log("Current:"+current);
			}
			
			// Discrete point
			if(model == 0) { 
				transform.position = current;
				yield return new WaitForSeconds(0.5f);
			}
			// Kinematic point
			else if(model == 1) {
				transform.position = Vector3.MoveTowards (transform.position, current, kinematic_vel * Time.deltaTime);
				yield return null;
			}
			// Dynamic point
			else if(model == 2) {
				Vector3 dir;

				float distanceToTarget=Vector3.Distance (current, transform.position);
				/*
				REAL CODE BELOW!!


					float neededDistToStop=(Mathf.Pow(velX,2)/2*accX);
					//Accelerate
					if(distanceToTarget>neededDistToStop){
						velX=velX+accX;
					}
					//Decelerate
					else{
						//Debug.Log("Decelerate");
						velX=velX-accX;
					}
					//Debug.Log("VelX:"+velX);
					//velY=velY+accY;
					dir = Vector3.Normalize (current - transform.position);
					//Since the vector is normalized it should be the same velocity for both
					dir.x = (dir.x * velX) * Time.deltaTime;
					dir.z = (dir.z * velX) *  Time.deltaTime;

					transform.position = (transform.position + dir);

				*/

				//Test Code


				dir=Vector3.Normalize(current-transform.position);

				Vector3 normVel=Vector3.Normalize(dynPVel);

				//The change is the difference between the direction and the velocity vector
				Vector3 change=Vector3.Normalize(dir-normVel);

				//Debug.Log ("Dir:"+dir);
				//Debug.Log("accMax:"+accMax);
				//Debug.Log ("DynVel:"+dynPVel);

				dynPVel.x=dynPVel.x+accMax*change.x*Time.deltaTime;
				dynPVel.z=dynPVel.z+accMax*change.z*Time.deltaTime;
			
				transform.position=transform.position+dynPVel;

				yield return null;
			}
			// Differential drive
			// Differential drive
			else if(model == 3) {
				Vector3 dir;
				Quaternion theta = Quaternion.LookRotation (current - transform.position);
				/*if(Vector3.Distance (current, transform.position) < vel*Time.deltaTime && theta==transform.rotation) {
					dir = current - transform.position;
					//Debug.Log("Jump");
					transform.position = current;
				}*/
				//else {

				//if(transform.rotation!=theta){
					
					
				//}
				//else{

					Vector3 curPos=transform.position;
					Vector3 rot=transform.eulerAngles;
				float curTheta=rot.y*(Mathf.PI/180);
					Vector3 dirPoint=transform.position;
					dirPoint.x=curPos.x+1*Mathf.Sin(curTheta);
					dirPoint.z=curPos.z+1*Mathf.Cos(curTheta);
					Vector3 lookDir=Vector3.Normalize(dirPoint-curPos);

				Debug.Log("LookDir:"+lookDir);

					dir = Vector3.Normalize (current - transform.position);	
				Debug.Log("Dir:"+dir);

				float diffAngle=Vector3.Angle(dir,lookDir);

				float stepsToRightAngle=diffAngle/(angleVel*Time.deltaTime);

				float stepsToDist=distance/(vel*Time.deltaTime);

				if(stepsToRightAngle>stepsToDist){
					transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, angleVel*Time.deltaTime);
				}
				else{

					transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, angleVel*Time.deltaTime);

					/*dir.x = dir.x * (  vel * Time.deltaTime);
					dir.z = dir.z * ( vel * Time.deltaTime);
					transform.position = (transform.position + dir);*/
				//}

				float curVel=0;

				//if(diffAngle<75){
					curVel=Vector3.Dot(dir,lookDir)*vel;
				//}

				Debug.Log(curVel);

					//dir = Vector3.Normalize (current - transform.position);					
				lookDir.x = lookDir.x * (  curVel * Time.deltaTime);
				lookDir.z = lookDir.z * ( curVel * Time.deltaTime);
					transform.position = (transform.position + lookDir);

				}
					//}
					/*
					dir = Vector3.Normalize (current - transform.position);
					Quaternion theta = Quaternion.LookRotation (current - transform.position);
					dir.x = dir.x + Mathf.Cos (theta.y) * vel * Time.deltaTime;
					dir.z = dir.z + Mathf.Sin (theta.y) * vel * Time.deltaTime;
					if(theta != transform.rotation) {
						transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, 1000 * Time.deltaTime);
					}
					else {
						transform.position = (transform.position + dir);
					}
					*/
				//}
				yield return null;
			}
			//Kinematic Car Model
			else if(model == 4){
				Vector3 dir;
				//Debug.Log("Inside Kinematic Car Model");
				float wheelAngleRad=maxWheelAngle*(Mathf.PI/180);
				float dTheta=(vel/carLength)*Mathf.Tan(wheelAngleRad);
				Quaternion theta = Quaternion.LookRotation (current - transform.position);
				/*if(Vector3.Distance (current, transform.position) < vel*Time.deltaTime ) {
					dir = current - transform.position;
					Debug.Log("Jump");
					transform.position = current;
				}*/
				//else {
					
					if(transform.rotation!=theta){
						transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, dTheta * Time.deltaTime);
					}

						//dir = Vector3.Normalize (current - transform.position);					
						//dir.x = dir.x * (  vel * Time.deltaTime);
						//dir.z = dir.z * ( vel * Time.deltaTime);

					Vector3 curDir=transform.eulerAngles;
					//Debug.Log("Euler Angles:"+transform.eulerAngles);
					Vector3 newPos=transform.position;
					float angleRad=curDir.y*(Mathf.PI/180);
					newPos.x=newPos.x+(vel*Mathf.Sin(angleRad)*Time.deltaTime);
					newPos.z=newPos.z+(vel*Mathf.Cos(angleRad)*Time.deltaTime);
					transform.position=newPos;
					//transform.position = (transform.position + newPos);

					//If the car is "almost" at the point
					if(Vector3.Distance (current, transform.position) < 5*vel*Time.deltaTime && 
					   (newPos.x==current.x || newPos.z==current.z)){
						Debug.Log("Car Almost at point");
						carMadeIt=true;
					}

			//}
				yield return null;
			}
			//Dynamic Car Model
			else if(model == 5){
				Vector3 dir;

				float distanceToTarget=Vector3.Distance (current, transform.position);


				float neededDistToStop=(Mathf.Pow(dynVel,2)/2*(dynF/dynMass));
				if(distanceToTarget>neededDistToStop){
					dynVel=dynVel+(dynF/dynMass);
				}
				else{
					dynVel=dynVel-(dynF/dynMass);
				}


				float wheelAngleRad=maxWheelAngle*(Mathf.PI/180);
				float dTheta=(dynVel/carLength)*Mathf.Tan(wheelAngleRad);
				Quaternion theta = Quaternion.LookRotation (current - transform.position);
				/*if(Vector3.Distance (current, transform.position) < dynVel*Time.deltaTime ) {
					dir = current - transform.position;
					Debug.Log("Jump");
					transform.position = current;
				}*/
				//else {
					
					if(transform.rotation!=theta){
						transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, dTheta * Time.deltaTime);
					}


					Vector3 curDir=transform.eulerAngles;
					//Debug.Log("Euler Angles:"+transform.eulerAngles);
					Vector3 newPos=transform.position;
					float angleRad=curDir.y*(Mathf.PI/180);
					newPos.x=newPos.x+(dynVel*Mathf.Sin(angleRad)*Time.deltaTime);
					newPos.z=newPos.z+(dynVel*Mathf.Cos(angleRad)*Time.deltaTime);
					transform.position=newPos;
					//transform.position = (transform.position + newPos);
					
					//If the car is "almost" at the point
					if(Vector3.Distance (current, transform.position) < 5*dynVel*Time.deltaTime && 
					   (newPos.x==current.x || newPos.z==current.z)){
						Debug.Log("Car Almost at point");
						carMadeIt=true;
					}
					
				//}
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
				Gizmos.color = Color.black;
				Gizmos.DrawCube (path[i], Vector3.one);
				
				if(i == 0) {
					//	Gizmos.DrawLine (transform.position, path[i]);
				}
				else {
					Gizmos.DrawLine (path[i-1], path[i]);
				}
			}
		}
	}
}
