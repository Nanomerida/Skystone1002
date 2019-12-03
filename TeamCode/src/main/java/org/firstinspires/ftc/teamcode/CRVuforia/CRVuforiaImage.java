package org.firstinspires.ftc.teamcode.CRVuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import com.vuforia.Matrix44F;
import com.vuforia.TrackableResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.vuforia.Tool;


public class CRVuforiaImage implements VuforiaTrackable.Listener{

  OpenGLMatrix stoneLocation;
  VectorF stoneTranslation;
  VuforiaTrackable trackable;
  boolean targetVisible;
  float x;
  float y;
  float z;

  public CRVuforiaImage(VuforiaTrackable trackable){
    this.trackable = trackable;
  }
  
  protected VectorF findImagePos(){
    try {
      stoneTranslation = stoneLocation.getTranslation();
    } catch (NullPointerException e){
      return null;
    }
    return stoneTranslation;
  }

  protected boolean imageVisible(){
    return targetVisible;
  }

  @Override
  public void onTracked(TrackableResult trackableResult, CameraName cameraName, Camera camera, VuforiaTrackable child){
    targetVisible = true;
    stoneLocation = new OpenGLMatrix(Tool.convertPose2GLMatrix(trackableResult.getPose()));
  }

  @Override
  public void onNotTracked(){
    targetVisible = false;
  }

  @Override
  public void addTrackable(VuforiaTrackable trackable){
    this.trackable = trackable;
  }


}

  
  
  
    
