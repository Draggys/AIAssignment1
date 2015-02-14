using UnityEngine;
using System.Collections;

public class RRTKinematicNode {


	public Vector3 position;
	public RRTKinematicNode parent;
	private float cost;

	public RRTKinematicNode(Vector3 pos){
		position = pos;
		parent = null;
		cost = 0;
		}

	public void setParent(RRTKinematicNode par){
		parent = par;
		}

	public void setCost(float cost){
		this.cost=cost;
	}

	public float getCost(){
		return cost;
	}


}
