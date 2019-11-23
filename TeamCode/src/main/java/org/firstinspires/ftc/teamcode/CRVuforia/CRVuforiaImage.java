package org.firstinspires.ftc.teamcode.CRVuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import com.vuforia.TrackableResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;


public class CRVuforiaImage extends VuforiaTrackableDefaultListener {

  OpenGLMatrix stoneLocation;
  VectorF stoneTranslation;
  float x;
  float y;
  float z;

  public CRVuforiaImage(VuforiaTrackable trackable){
    super(trackable);
  }
  
  protected VectorF findImagePos(){
    stoneLocation = super.getVuforiaCameraFromTarget();
    try {
      stoneTranslation = stoneLocation.getTranslation();
    } catch (NullPointerException e){
      return null;
    }
    x = stoneTranslation.get(0);
    y = stoneTranslation.get(1);
    z = stoneTranslation.get(2);
    return stoneTranslation;
  }

  protected boolean imageVisible(){ return super.isVisible();}


}

  
  
  
    
