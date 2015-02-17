using UnityEngine;
using System.Collections;

public class RRTDynCarSteerRet {
	public float steps=0;
	public Quaternion rotation;
	public float dynVel = 0;
	
	public RRTDynCarSteerRet(float steps, Quaternion rotation, float vel){
		this.steps = steps;
		this.rotation = rotation;
		this.dynVel = vel;
	}
	
	
	public float getSteps(){
		return this.steps;
	}
	
	public Quaternion getRot(){
		return this.rotation;
	}

	public float getDynVel(){
		return this.dynVel;
	}

}


