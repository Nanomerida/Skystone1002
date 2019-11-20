package org.firstinspires.ftc.teamcode.Mecanum;

import org.firstinspires.ftc.teamcode.Mecanum.TwoDPosition;
import org.firstinspires.ftc.teamcode.Mecanum.TwoDOrientation;


public class RobotPosition {
  
  TwoDPosition position;
  TwoDOrientation heading;

  public RobotPosition(double x, double y, double heading, String headingUnit) {
     this.position = new TwoDPosition(x, y);
     this.heading = new TwoDOrientation(heading, headingUnit);
  }
 
 
  public double[] getPosition(){
    return position.getCurrentPosition();
  }
  
  public void setPosition(double x, double y){
    position.setCurrentPosition(x, y);
  }
  
  public double getHeading(){
    heading.getHeading();
  }
  
  public void setHeading (double heading){
    heading.setHeading(heading);
  }
}
