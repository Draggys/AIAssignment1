using UnityEngine;
using System.Collections;

public class RRTKinCarSteerRet {
	public float steps=0;
	public Quaternion rotation;
	
	public RRTKinCarSteerRet(float steps, Quaternion rotation){
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
