using UnityEngine;
using System.Collections;


public class RRTDynPSteerRet {

	public int steps=0;
	public Vector3 velocity;

	public RRTDynPSteerRet(int steps, Vector3 velocity){
		this.steps = steps;
		this.velocity = velocity;
	}


	public int getSteps(){
		return this.steps;
	}

	public Vector3 getVel(){
		return this.velocity;
	}


}