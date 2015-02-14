using UnityEngine;
using System.Collections;

public class RRTDynamicPNode {
	
	
	public Vector3 position;
	public RRTDynamicPNode parent;
	public Vector3 velocity;
	private float cost;
	
	public RRTDynamicPNode(Vector3 pos){
		position = pos;
		parent = null;
		cost = 0;
	}
	
	public void setParent(RRTDynamicPNode par){
		parent = par;
	}
	
	public void setCost(float cost){
		this.cost=cost;
	}
	
	public float getCost(){
		return cost;
	}

	public Vector3 getVelocity(){
		return velocity;
	}

	public void setVelocity(Vector3 vel){
		this.velocity = vel;
	}
	
}
