using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RRTDynamicPoint : MonoBehaviour{
	
	List<RRTDynamicPNode> points;
	public float xLow;
	public float xHigh;
	public float zLow;
	public float zHigh;
	
	public float goalInterval;
	public int nrIterations;
	public float nearRadius;
	
	public float accMax;
	
	private PolyMapLoader map;
	
	
	private List<Vector3> path = null;
	
	void Start(){
		
		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");
		
		points = new List<RRTDynamicPNode> ();
		points.Add (new RRTDynamicPNode (map.polyData.start));
		
		Debug.Log ("Starting RRT");
		
		this.doRRT (nrIterations, map.polyData.end);
		
		if (path != null) {
			StartCoroutine("Move");
		}
		
	}
	
	
	public void doRRT(int nrIterations, Vector3 endPoint){
		
		for (int i=0; i<nrIterations; i++) {
			
			Vector3 curRand=this.sampleFree();
			
			RRTDynamicPNode nearest=this.closestPoint(curRand);
			
			//If obstacle free path
			RRTDynPSteerRet steerRet=this.steer(nearest.position,curRand,nearest.getVelocity());

			if(steerRet!=null){
				
				RRTDynamicPNode newNode=new RRTDynamicPNode(curRand);
				
				List<RRTDynamicPNode> nearNodes=this.getNearPoints(newNode);
				
				RRTDynamicPNode xmin=nearest;
				float cmin=nearest.getCost()+steerRet.getSteps();
				Vector3 newVel=steerRet.getVel();
				
				//Check is path with less cost exists to new node
				foreach(RRTDynamicPNode near in nearNodes){
					//If collision free && lesser cost
					RRTDynPSteerRet ret=this.steer(near.position,curRand,near.getVelocity());
					if(ret!=null){
					float thisCost=near.getCost()+ret.getSteps();
					if(thisCost<cmin){
						xmin=near;
						cmin=thisCost;
						newVel=ret.getVel();
					}
				}
				}
				newNode.setParent(xmin);
				newNode.setCost(cmin);
				newNode.setVelocity(newVel);
				points.Add(newNode);
				
				//Check if any of near points can be rewired
				foreach(RRTDynamicPNode near in nearNodes){

					RRTDynPSteerRet ret=this.steer(newNode.position,near.position,newNode.getVelocity());

					if(ret!=null){
					float costThroughNew=newNode.getCost()+ret.getSteps();
					
						if(costThroughNew<near.getCost()){
							near.setParent(newNode);
							near.setCost(costThroughNew);
							near.velocity=ret.getVel();
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
			
			RRTDynamicPNode nearest=this.closestPoint(curRand);
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
	
	
	
	/*
	 * Find the closest point among the old points 
	 */
	private RRTDynamicPNode closestPoint(Vector3 newPoint){
		
		float curLowest = float.PositiveInfinity;
		RRTDynamicPNode curNearest = null;
		
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
	private List<RRTDynamicPNode> getNearPoints(RRTDynamicPNode newNode){
		
		List<RRTDynamicPNode> nearNodes = new List<RRTDynamicPNode> ();
		
		for (int i=0; i<points.Count; i++) {
			
			float dist=Vector3.Distance(points[i].position,newNode.position);
			if(dist<nearRadius){
				nearNodes.Add(points[i]);
			}
			
		}
		return nearNodes;
		
	}
	
	//Function to "steer" from start point to end point
	private RRTDynPSteerRet steer(Vector3 start,Vector3 end, Vector3 startVel){
		
		Vector3 dynPVel = startVel;

		Vector3 curPosition = start;

		Vector3 goalPosition = end;

		float startDistance = Vector3.Distance (start, end);

		int nrSteps = 0;

		while (true) {
			float distance=Vector3.Distance(curPosition,goalPosition);
			if(distance<=goalInterval && distance<(startDistance/2)) {
				//If we have arrived at out end point
				return new RRTDynPSteerRet(nrSteps,dynPVel);
			}
			
			Vector3 dir;
			
			float distanceToTarget=Vector3.Distance (goalPosition, curPosition);
			
			dir=Vector3.Normalize(goalPosition-curPosition);
			
			Vector3 normVel=Vector3.Normalize(dynPVel);
			
			//The change is the difference between the direction and the velocity vector
			Vector3 change=Vector3.Normalize(dir-normVel);
			
			//Debug.Log ("Dir:"+dir);
			//Debug.Log("accMax:"+accMax);
			//Debug.Log ("DynVel:"+dynPVel);
			
			dynPVel.x=dynPVel.x+accMax*change.x;
			dynPVel.z=dynPVel.z+accMax*change.z;
			
			Vector3 newPosition=curPosition+dynPVel;

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
		
		RRTDynamicPNode goalNode = null;
		List<Vector3> path=new List<Vector3>();
		
		foreach (RRTDynamicPNode node in points) {
			
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
		
		RRTDynamicPNode curNode = goalNode;
		
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

		Vector3 dynPVel = new Vector3 (0, 0, 0);

		Vector3 current = path[index];
		float timeBefore = Time.time;
		while (true) {
			float distance=Vector3.Distance(transform.position,current);
			float curVel=Vector3.Magnitude(dynPVel);
			if(distance<=goalInterval*curVel ) {
				index++;
				
				//Debug.Log("Arrived at position");
				if(index >= path.Count) {
					float timeAfter=Time.time;
					Debug.Log("Time:"+(timeAfter-timeBefore));
					yield break;
				}
				current = path[index];
				//Debug.Log("Current:"+current);
			}

			Vector3 dir;
			
			float distanceToTarget=Vector3.Distance (current, transform.position);

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

