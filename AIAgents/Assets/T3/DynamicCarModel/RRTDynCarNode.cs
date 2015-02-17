using UnityEngine;
using System.Collections;

public class RRTDynCarNode {
	
	public Vector3 position;
	public RRTDynCarNode parent;
	public Quaternion direction;
	private float cost;
	private float dynVel;
	
	public RRTDynCarNode(Vector3 pos){
		position = pos;
		parent = null;
		cost = 0;
		dynVel = 0;
	}
	
	public void setParent(RRTDynCarNode par){
		parent = par;
	}
	
	public void setCost(float cost){
		this.cost=cost;
	}

	public void setDynVel(float vel){
		dynVel = vel;
	}

	public float getDynVel(){
		return dynVel;
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
