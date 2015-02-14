using UnityEngine;
using System.Collections;


public class RRTDiffSteerRet {
	
	public float steps=0;
	public Quaternion rotation;
	
	public RRTDiffSteerRet(float steps, Quaternion rotation){
		this.steps = steps;
		this.rotation = rotation;
	}
	
	
	public float getSteps(){
		return this.steps;
	}
	
	public Quaternion getRot(){
		return this.rotation;
	}
	
	
}