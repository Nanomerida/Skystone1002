package org.firstinspires.ftc.teamcode.CRVuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class CRVuforiaImage {

  OpenGLMatrix stoneLocation;
  VectorF stoneTranslation;
  float x;
  float y;
  float x;

  public CRVuforiaImage(){
    
  }
  
  public void setImagePos(OpenGLMatrix newPos){
    this.stoneLocation = newPos;
    stoneTranslation = stoneLocation.getTranslation();
    x = stoneTranslation.get(0);
    y = stoneTranslation.get(1);
    z = stoneTranslation.get(2);
  }
  public VectorF getImageTranslation(){
    return stoneTranslation;
  }
  public float getImageX(){ return x; }
  public float getImageY(){ return y; }
  public float getImageZ(){ return z; }
}
  
  
  
    
