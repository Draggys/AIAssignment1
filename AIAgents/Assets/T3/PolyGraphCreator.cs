using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PolyGraphCreator {
	
	public List<Line> possibleLines;

	public PolyGraphCreator(PolyData data){


		possibleLines = new List<Line> ();

		this.findLines (data);

		}


	public void findLines(PolyData data){

		//For every figure, iterate through the vertices and check if it's possible
		//to draw a line to a vertice in any other figure
		for (int i=0; i<data.figures.Count; i++) {
			PolyNode curFig=data.figures[i];
			foreach(Vector3 curVertice in curFig.vertices){

				for(int j=0;j<data.figures.Count;i++){
					//If it's the same figure continue;
					if(i==j){
						continue;
					}

					PolyNode curTestFigure=data.figures[j];

					foreach(Vector3 endVertice in curTestFigure.vertices){
						Line testLine=new Line(curVertice,endVertice);
						if(!this.doIntersect(data,testLine)){
							possibleLines.Add(testLine);
						}
					}
				}

			}
		}


		}

	//Checks if a line intersect any obstacle line
	private bool doIntersect(PolyData data, Line testLine){

		foreach (Line curLine in data.lines) {

				if(testLine.intersect(curLine)){
				//Debug.Log("TestLine:"+testLine.point1+" "+testLine.point2);
				//Debug.Log("Obs line:"+curLine.point1+" "+curLine.point2);

					return true;
				}

				}
		return false;
	}



}
