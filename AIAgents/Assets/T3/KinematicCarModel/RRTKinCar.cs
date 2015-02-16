using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RRTKinCar : MonoBehaviour {

	public Dubin dubin;

	public float xLow;
	public float xHigh;
	public float zLow;
	public float zHigh;
	
	public float goalInterval;
	public int nrIterations;
	public float nearRadius;
	
	public float carLength = 4f;
	public float maxWheelAngle = 87f;
	public float vel = 1;
	float minRadius = 0f;

	List<Vector3> oldPath = null;
	List<Vector3> path = null;
	List<RRTKinCarNode> points;

	private PolyMapLoader map;

	bool isRunning = false;

	void Start() {
		minRadius = Mathf.Abs (carLength / (Mathf.Tan (maxWheelAngle) ));
		dubin = gameObject.AddComponent<Dubin> ();
		
		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");
		points = new List<RRTKinCarNode> ();
		RRTKinCarNode startNode = new RRTKinCarNode (map.polyData.start);
		startNode.direction = transform.rotation;
		points.Add (startNode);

		doRRT (nrIterations, map.polyData.end);

		print ("#Points: " + points.Count);

//		path = new List<Vector3> ();
//		path.Add (transform.position);
//		path.Add (new Vector3 (5, 1, 46));
//		path.Add (new Vector3 (5, 1, 6));
//		path.Add (new Vector3 (87, 1, 6));
//		path.Add (new Vector3 (80, 1, 30));


		if (path != null) {
			List<Vector3> newPath = new List<Vector3> ();
			for(int i = 0; i < path.Count; i++) {
				newPath.Add (path[i]);
				if(checkIntersection (path[i], path[path.Count - 1])) {
					// not intersection
					newPath.Add (path[path.Count - 1]);
					break;
				}
			}
			
			print ("Old path: " + path.Count);
			print ("New path: " + newPath.Count);
			
			oldPath = path;
			path = newPath;
			StartCoroutine("Move");
		}


		/*
		// OBS 
		// This won't work. We need to steer again and not use lines.
		if (path != null) {
			List<Vector3> newPath = new List<Vector3> ();
			int i = 1;
			newPath.Add (path[0]);
			while(i < path.Count) {
				// This won't work. We need to steer again and not use lines.
				if(!checkIntersection (newPath[newPath.Count - 1], path[i])) {
					// intersection
					newPath.Add (path[i - 1]);
				}
				i++;
			}
			newPath.Add (path[path.Count - 1]);

			oldPath = path;
			path = newPath;

			StartCoroutine("Move");
		}
		*/
	}



	public void doRRT(int nrIterations, Vector3 endPoint){
		
		for (int i=0; i<nrIterations; i++) {
			
			//Debug.Log ("Iteration: " +i);
			
			Vector3 curRand=this.sampleFree();
			
			RRTKinCarNode nearest=this.closestPoint(curRand);
			
			//If obstacle free path
			RRTKinCarSteerRet steerRet=this.steer(nearest.position,curRand,nearest.getDirection(), nearest.getDirection ()); //endrot?
			
			if(steerRet!=null){
				
				RRTKinCarNode newNode=new RRTKinCarNode(curRand);
				
				List<RRTKinCarNode> nearNodes=this.getNearPoints(newNode);
				
				RRTKinCarNode xmin=nearest;
				float cmin=nearest.getCost()+steerRet.getSteps();
				Quaternion newRot=steerRet.getRot();
				
				//Check is path with less cost exists to new node
				foreach(RRTKinCarNode near in nearNodes){
					//If collision free && lesser cost
					RRTKinCarSteerRet ret=this.steer(near.position,curRand,near.getDirection(), near.getDirection ()); //randomize end pos?
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
				foreach(RRTKinCarNode near in nearNodes){
					
					RRTKinCarSteerRet ret=this.steer(newNode.position,near.position,newNode.getDirection(), newNode.getDirection ()); //endRot?
					
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

	private RRTKinCarSteerRet steer(Vector3 start,Vector3 end, Quaternion startRot, Quaternion endRot){
		float r1 = minRadius, r2 = minRadius;
		DubinRet S = dubin.MinTrajectory (start, end, startRot, endRot, r1, r2);

		if (S == null)
			return null;

		dS = new Line (S.waypoints [0], S.waypoints [1]);

		List<Vector3> path = new List<Vector3> ();
		path.Add (start);
		path.Add (end);

		Vector3 curPos = start;
		Vector3 goalPos = end;
		Quaternion curRot = startRot;

		bool followS = true;
		int q = 2;
		bool carMadeIt = false;
		int index = 0;
		Vector3 current = path [index];
		int nrSteps = 0;
		while (true) {
			if(carMadeIt) {
				if(Vector3.Distance (curPos, path[index]) < goalInterval) {
					index++;

					if(index >= path.Count){
						return new RRTKinCarSteerRet(S.cost, curRot);
						//return new RRTKinCarSteerRet(nrSteps, curRot);
					}

					followS = true;
					current = path[index];
				}
			}
			if (followS) {
				if(q == 2) {
					current = S.waypoints[0];
					if(Vector3.Distance (curPos, current) < goalInterval) {
						q--;
					}
				}
				if(q == 1) {
					current = S.waypoints[1];
					if(Vector3.Distance(curPos, current) < goalInterval) {
						q--;
					}
				}
				if(q == 0) {
					followS = false;
					current = path[index];
				}
			}

			float wheelAngleRad = maxWheelAngle * (Mathf.PI / 180);
			float dTheta = (vel / carLength) * Mathf.Tan (wheelAngleRad);
			Quaternion theta = Quaternion.LookRotation(current - curPos);

			if(curRot != theta) {
				curRot = Quaternion.RotateTowards(curRot, theta, dTheta);
			}

			Vector3 curDir = curRot.eulerAngles;
			Vector3 newPos = curPos;
			float angleRad = curDir.y * (Mathf.PI / 180);
			newPos.x = newPos.x + (vel * Mathf.Sin (angleRad) * Time.deltaTime);
			newPos.z = newPos.z + (vel * Mathf.Cos (angleRad) * Time.deltaTime);

			if(!checkIntersection (curPos, newPos) && curPos != newPos) {
				return null;
			}
				
			if(Vector3.Distance (current, curPos) < goalInterval) {
				carMadeIt = true;
			}


			curPos = newPos;
			nrSteps++;
		}
	}
	
	private Vector3 sampleFree(){
		
		Vector3 curRand=new Vector3(Random.Range(xLow,xHigh),1,Random.Range(zLow,zHigh));
		bool valid = false;
		
		while (!valid) {
			
			RRTKinCarNode nearest=this.closestPoint(curRand);
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

	private RRTKinCarNode closestPoint(Vector3 newPoint){
		
		float curLowest = float.PositiveInfinity;
		RRTKinCarNode curNearest = null;
		
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
	private List<RRTKinCarNode> getNearPoints(RRTKinCarNode newNode){
		
		List<RRTKinCarNode> nearNodes = new List<RRTKinCarNode> ();
		
		for (int i=0; i<points.Count; i++) {
			
			float dist=Vector3.Distance(points[i].position,newNode.position);
			if(dist<nearRadius){
				nearNodes.Add(points[i]);
			}
			
		}
		return nearNodes;
		
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
		
		RRTKinCarNode goalNode = null;
		List<Vector3> path=new List<Vector3>();
		
		foreach (RRTKinCarNode node in points) {
			
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
		
		RRTKinCarNode curNode = goalNode;
		
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

	void OnDrawGizmos() {
        
		if (moving) {
			Gizmos.color = Color.black;
			if (dubin.proxCircles != null) {
				foreach (Circle circle in dubin.proxCircles) {
					Gizmos.DrawWireSphere (circle.pos, minRadius);
				}
			}
        
			Gizmos.color = Color.magenta;
			if (oldPath != null) {
				foreach (Vector3 pos in oldPath) {
					Gizmos.DrawCube (pos, Vector3.one * 2);
				}
			}
			
			Gizmos.color = Color.white;
			if (path != null) {
				foreach (Vector3 pos in path) {
					Gizmos.DrawCube (pos, Vector3.one);
				}
			}		
			Gizmos.color = Color.green;
			if (dS != null) {
				Gizmos.DrawLine (dS.point1, dS.point2);
			}
		}
        
        /*
		Gizmos.color = Color.blue;
		if (dubin.tangents != null) {
			foreach(Line line in dubin.tangents) {
				Gizmos.DrawLine (line.point1, line.point2);
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
		*/
    }

	int index = 1;
	Vector3 current = Vector3.zero;
	Line dS = null;
	bool moving = false;
	IEnumerator Move() {
		moving = true;
		current = path [index];
		
		DubinRet S = dubin.MinTrajectory (transform.position, current, transform.rotation, 
		                                       transform.rotation, minRadius, minRadius);
		
		dS = new Line(S.waypoints[0], S.waypoints[1]);
		
		bool followS = true;
		int q = 2;
		bool carMadeIt = false;
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

}
