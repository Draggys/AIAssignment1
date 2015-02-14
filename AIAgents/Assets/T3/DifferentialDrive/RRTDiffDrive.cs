using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RRTDiffDrive : MonoBehaviour{
	
	List<RRTDiffNode> points;
	public float xLow;
	public float xHigh;
	public float zLow;
	public float zHigh;
	
	public float goalInterval;
	public int nrIterations;
	public float nearRadius;
	
	public float vMax;
	public float wMax;
	private float wMaxDegrees;
	
	private PolyMapLoader map;
	
	
	private List<Vector3> path = null;
	
	void Start(){
		
		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");
		
		points = new List<RRTDiffNode> ();
		RRTDiffNode startNode = new RRTDiffNode (map.polyData.start);
		startNode.direction = transform.rotation;
		points.Add (startNode);
		
		Debug.Log ("Starting RRT");

		wMaxDegrees=wMax*(180/Mathf.PI);
		
		this.doRRT (nrIterations, map.polyData.end);
		
		if (path != null) {
			StartCoroutine("Move");
		}
		
	}
	
	
	public void doRRT(int nrIterations, Vector3 endPoint){
		
		for (int i=0; i<nrIterations; i++) {

			//Debug.Log ("Iteration: " +i);
			
			Vector3 curRand=this.sampleFree();
			
			RRTDiffNode nearest=this.closestPoint(curRand);
			
			//If obstacle free path
			RRTDiffSteerRet steerRet=this.steer(nearest.position,curRand,nearest.getDirection());
			
			if(steerRet!=null){
				
				RRTDiffNode newNode=new RRTDiffNode(curRand);
				
				List<RRTDiffNode> nearNodes=this.getNearPoints(newNode);
				
				RRTDiffNode xmin=nearest;
				float cmin=nearest.getCost()+steerRet.getSteps();
				Quaternion newRot=steerRet.getRot();
				
				//Check is path with less cost exists to new node
				foreach(RRTDiffNode near in nearNodes){
					//If collision free && lesser cost
					RRTDiffSteerRet ret=this.steer(near.position,curRand,near.getDirection());
					if(ret!=null){
						float thisCost=near.getCost()+ret.getSteps();
						if(thisCost<cmin){
							xmin=near;
							cmin=thisCost;
							newRot=ret.getRot();
						}
					}
				}
				newNode.setParent(xmin);
				newNode.setCost(cmin);
				newNode.setDirection(newRot);
				points.Add(newNode);
				
				//Check if any of near points can be rewired
				foreach(RRTDiffNode near in nearNodes){
					
					RRTDiffSteerRet ret=this.steer(newNode.position,near.position,newNode.getDirection());
					
					if(ret!=null){
						float costThroughNew=newNode.getCost()+ret.getSteps();
						
						if(costThroughNew<near.getCost()){
							near.setParent(newNode);
							near.setCost(costThroughNew);
							near.direction=ret.getRot();
						}
					}
				}
				
			}
			
		}
		
		path=this.findPath (map.polyData.end);
		if (path == null) {
			Debug.Log ("No path found");
		} 
		else {
			Debug.Log("Path found");
		}
		
	}
	
	private Vector3 sampleFree(){
		
		Vector3 curRand=new Vector3(Random.Range(xLow,xHigh),1,Random.Range(zLow,zHigh));
		bool valid = false;
		
		while (!valid) {
			
			RRTDiffNode nearest=this.closestPoint(curRand);
			if(this.checkIntersection(nearest.position,curRand)){
				valid=true;
			}
			else{
				curRand.x=Random.Range(xLow,xHigh);
				curRand.z=Random.Range(zLow,zHigh);
			}
			
		}
		
		return curRand;
		
	}
	
	
	

	 // Find the closest point among the old points 

	private RRTDiffNode closestPoint(Vector3 newPoint){
		
		float curLowest = float.PositiveInfinity;
		RRTDiffNode curNearest = null;
		
		for (int i=0; i<points.Count; i++) {
			
			float dist=Vector3.Distance(points[i].position,newPoint);
			if(dist<curLowest){
				curLowest=dist;
				curNearest=points[i];
			}
		}
		return curNearest;
	}
	
	//To get the list of near points needed in RRT*
	private List<RRTDiffNode> getNearPoints(RRTDiffNode newNode){
		
		List<RRTDiffNode> nearNodes = new List<RRTDiffNode> ();
		
		for (int i=0; i<points.Count; i++) {
			
			float dist=Vector3.Distance(points[i].position,newNode.position);
			if(dist<nearRadius){
				nearNodes.Add(points[i]);
			}
			
		}
		return nearNodes;
		
	}
	
	//Function to "steer" from start point to end point
	private RRTDiffSteerRet steer(Vector3 start,Vector3 end, Quaternion startRot){

		/*Quaternion theta = Quaternion.LookRotation (end - start);

		Line newLine = new Line (start, end);
		
		foreach (Line line in map.polyData.lines) {
			
			if(newLine.intersect(line)){
				return null;
			}
			
		}

		float dist = Vector3.Distance (start, end);

		return new RRTDiffSteerRet(dist,theta);*/

		Vector3 curPosition = start;
		
		Vector3 goalPosition = end;
		
		float startDistance = Vector3.Distance (start, end);
		
		int nrSteps = 0;

		Quaternion curRot = startRot;
		
		while (true) {
			float distance=Vector3.Distance(curPosition,goalPosition);
			if(distance<=goalInterval && distance<(startDistance/2)) {
				//If we have arrived at out end point
				return new RRTDiffSteerRet(nrSteps,curRot);
			}
			
			Vector3 dir;
			Quaternion theta = Quaternion.LookRotation (goalPosition - curPosition);
			
			//Vector3 curPos=transform.position;
			//Vector3 rot=transform.eulerAngles;
			float curTheta=curRot.eulerAngles.y*(Mathf.PI/180);
			Vector3 dirPoint=curPosition;
			dirPoint.x=curPosition.x+1*Mathf.Sin(curTheta);
			dirPoint.z=curPosition.z+1*Mathf.Cos(curTheta);
			Vector3 lookDir=Vector3.Normalize(dirPoint-curPosition);
			
			dir = Vector3.Normalize (goalPosition - curPosition);	
			
			float diffAngle=Vector3.Angle(dir,lookDir);
			
			float stepsToRightAngle=diffAngle/(wMaxDegrees);
			
			float stepsToDist=distance/(vMax);

			Vector3 newPosition=curPosition;

			if(stepsToRightAngle>stepsToDist){
				curRot = Quaternion.RotateTowards (curRot, theta, wMaxDegrees);
			}
			else{
				
				curRot = Quaternion.RotateTowards (curRot, theta, wMaxDegrees);

				if(diffAngle<90){
					//float moveVel=Vector3.Dot(dir,lookDir)*vMax;
					//lookDir.x = lookDir.x * (  moveVel );
					//lookDir.z = lookDir.z * ( moveVel );
					lookDir.x = lookDir.x * (  vMax );
					lookDir.z = lookDir.z * ( vMax );
					newPosition=curPosition+lookDir;
				}
			}

			
			//Every time we have moved we check to see that we havent crossed any obstacle
			if(!this.checkIntersection(curPosition,newPosition)){
				return null;
			}
			
			curPosition=newPosition;
			nrSteps++;
		}
	}
	
	private bool checkIntersection(Vector3 start, Vector3 end){
		
		Line newLine = new Line (start, end);
		
		foreach (Line line in map.polyData.lines) {
			
			if(newLine.intersect(line)){
				return false;
			}
			
		}
		
		return true;
		
	}
	
	private List<Vector3> findPath(Vector3 endPoint){
		
		RRTDiffNode goalNode = null;
		List<Vector3> path=new List<Vector3>();
		
		foreach (RRTDiffNode node in points) {
			
			float distToGoal=Vector3.Distance(node.position,endPoint);
			
			//If there is a point close to the goal
			if(distToGoal<goalInterval){
				goalNode=node;
				Debug.Log("Path cost:"+goalNode.getCost());
				break;
			}
		}
		//If no goal node was found
		if(goalNode==null){
			return null;
		}
		
		RRTDiffNode curNode = goalNode;
		
		while (curNode.parent!=null) {
			
			path.Add(curNode.position);
			curNode=curNode.parent;
			
		}
		
		//Also add the startnode, which will be the node with parent==null
		path.Add (curNode.position);
		
		//Reverse the path
		path.Reverse();
		
		return path;
		
	}
	
	
	public bool hasPath(){
		return path!=null;
	}
	
	IEnumerator Move() {
		int index = 0;
		float timeBefore=Time.time;
		int steps = 0;
		Vector3 current = path[index];
		while (true) {
			float distance=Vector3.Distance(transform.position,current);

			if(distance<=goalInterval*vMax ) {
				index++;
				
				//Debug.Log("Arrived at position");
				if(index >= path.Count) {
					float timeAfter=Time.time;
					Debug.Log("Time:"+(timeAfter-timeBefore));
					Debug.Log("TimeSteps:"+steps);
					yield break;
				}
				current = path[index];
				//Debug.Log("Current:"+current);
			}

			Vector3 dir;
			Quaternion theta = Quaternion.LookRotation (current - transform.position);
			
			Vector3 curPos=transform.position;
			Vector3 rot=transform.eulerAngles;
			float curTheta=rot.y*(Mathf.PI/180);
			Vector3 dirPoint=transform.position;
			dirPoint.x=curPos.x+1*Mathf.Sin(curTheta);
			dirPoint.z=curPos.z+1*Mathf.Cos(curTheta);
			Vector3 lookDir=Vector3.Normalize(dirPoint-curPos);
			
			dir = Vector3.Normalize (current - transform.position);	
			
			float diffAngle=Vector3.Angle(dir,lookDir);
			
			float stepsToRightAngle=diffAngle/(wMaxDegrees);
			
			float stepsToDist=distance/(vMax);
			
			if(stepsToRightAngle>stepsToDist){
				transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, wMaxDegrees);
			}
			else{
				
				transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, wMaxDegrees);

				
				if(diffAngle<90){
					//float moveVel=Vector3.Dot(dir,lookDir)*vMax;					
					//lookDir.x = lookDir.x * (  moveVel );
					//lookDir.z = lookDir.z * ( moveVel );

					lookDir.x = lookDir.x * (  vMax );
					lookDir.z = lookDir.z * ( vMax );
					transform.position = (transform.position + lookDir);
				}

				
			}
			yield return null;



			/*
			Vector3 dir;

			Quaternion theta = Quaternion.LookRotation (current - transform.position);

			//else {

			if(transform.rotation!=theta){

				transform.rotation = Quaternion.RotateTowards (transform.rotation, theta, wMaxDegrees);
			}
			else{

				dir = Vector3.Normalize (current - transform.position);					
				dir.x = dir.x * (  vMax );
				dir.z = dir.z * ( vMax );
				transform.position = (transform.position + dir);
			}
			steps++;
			yield return null;*/
		}
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
		Gizmos.DrawCube (map.polyData.start, new Vector3(2,1,2));
		
		Gizmos.color = Color.red;
		Gizmos.DrawCube (map.polyData.end, new Vector3(2,1,2));
		
		Gizmos.color = Color.black;
		Gizmos.DrawLine (new Vector3 (0, 1, 0), new Vector3 (100, 1, 0));
		Gizmos.DrawLine (new Vector3 (100, 1, 0),new Vector3(100,1,90));
		Gizmos.DrawLine (new Vector3(100,1,90), new Vector3 (0, 1, 90));
		Gizmos.DrawLine (new Vector3(0,1,90), new Vector3 (0, 1, 0));
		
	}
	
	
}

