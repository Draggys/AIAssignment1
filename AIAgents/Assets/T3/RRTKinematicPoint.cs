using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RRTKinematicPoint : MonoBehaviour{

	List<RRTKinematicNode> points;
	public float xLow;
	public float xHigh;
	public float zLow;
	public float zHigh;

	public float goalInterval;
	public int nrIterations;
	public float nearRadius;

	public float velocity;

	private PolyMapLoader map;


	private List<Vector3> path = null;

	void Start(){

		map = new PolyMapLoader ("x", "y", "goalPos", "startPos", "button");

		points = new List<RRTKinematicNode> ();
		points.Add (new RRTKinematicNode (map.polyData.start));

		Debug.Log ("Starting RRT");

		this.doRRT (nrIterations, map.polyData.end);

		if (path != null) {
			StartCoroutine("Move");
			}

		}
	

	public void doRRT(int nrIterations, Vector3 endPoint){

		for (int i=0; i<nrIterations; i++) {

			Vector3 curRand=this.sampleFree();

			RRTKinematicNode nearest=this.closestPoint(curRand);

			//If obstacle free path
			if(this.steer(nearest.position,curRand)){

				RRTKinematicNode newNode=new RRTKinematicNode(curRand);

				List<RRTKinematicNode> nearNodes=this.getNearPoints(newNode);

				RRTKinematicNode xmin=nearest;
				float cmin=nearest.getCost()+Vector3.Distance(nearest.position,curRand);

				//Check is path with less cost exists to new node
				foreach(RRTKinematicNode near in nearNodes){
					//If collision free && lesser cost
					float thisCost=near.getCost()+Vector3.Distance(near.position,curRand);
					if(this.steer(near.position,curRand) && thisCost<cmin){
						xmin=near;
						cmin=thisCost;
					}
				}
				newNode.setParent(xmin);
				newNode.setCost(cmin);
				points.Add(newNode);

				//Check if any of near points can be rewired
				foreach(RRTKinematicNode near in nearNodes){

					float costThroughNew=newNode.getCost()+Vector3.Distance(newNode.position,near.position);

					if(this.steer(near.position,newNode.position) && costThroughNew<near.getCost()){
						near.setParent(newNode);
						near.setCost(costThroughNew);
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

			RRTKinematicNode nearest=this.closestPoint(curRand);
			if(this.steer(nearest.position,curRand)){
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
	private RRTKinematicNode closestPoint(Vector3 newPoint){

		float curLowest = float.PositiveInfinity;
		RRTKinematicNode curNearest = null;

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
	private List<RRTKinematicNode> getNearPoints(RRTKinematicNode newNode){

		List<RRTKinematicNode> nearNodes = new List<RRTKinematicNode> ();

		for (int i=0; i<points.Count; i++) {

			float dist=Vector3.Distance(points[i].position,newNode.position);
			if(dist<nearRadius){
				nearNodes.Add(points[i]);
			}

		}
		return nearNodes;

		}

	//Function to "steer" from start point to end point
	private bool steer(Vector3 start,Vector3 end){

		//For the kinematic point there is a obstacle free path if there is a line
		//between the points that doesn't cut any obstacle line

		Line newLine = new Line (start, end);

		foreach (Line line in map.polyData.lines) {

			if(newLine.intersect(line)){
				return false;
			}

				}

		return true;
		}

	private List<Vector3> findPath(Vector3 endPoint){

		RRTKinematicNode goalNode = null;
		List<Vector3> path=new List<Vector3>();

		foreach (RRTKinematicNode node in points) {

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

		RRTKinematicNode curNode = goalNode;

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
		float timeBefore = Time.time;
		Vector3 current = path[index];
		while (true) {
			float distance=Vector3.Distance(transform.position,current);
			if(distance<=goalInterval*Time.deltaTime ) {
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
			transform.position = Vector3.MoveTowards (transform.position, current, velocity );
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
