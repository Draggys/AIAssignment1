using UnityEngine;
using System.Collections;

public class RRTDiffNode {
	
	
	public Vector3 position;
	public RRTDiffNode parent;
	public Quaternion direction;
	private float cost;
	
	public RRTDiffNode(Vector3 pos){
		position = pos;
		parent = null;
		cost = 0;
	}
	
	public void setParent(RRTDiffNode par){
		parent = par;
	}
	
	public void setCost(float cost){
		this.cost=cost;
	}
	
	public float getCost(){
		return cost;
	}
	
	public Quaternion getDirection(){
		return direction;
		}

	public void setDirection(Quaternion dir){
		direction = dir;
	}
	
}
