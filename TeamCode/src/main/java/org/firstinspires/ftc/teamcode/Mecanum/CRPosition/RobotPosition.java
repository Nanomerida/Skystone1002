package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;



public class RobotPosition {
  
  TwoDPosition position;
  TwoDOrientation heading;

  public RobotPosition(double x, double y, double heading, String headingUnit) {
     this.position = new TwoDPosition(x, y);
     this.heading = new TwoDOrientation(heading, headingUnit);
  }

  /**
   * Returns a double array with the robot's position as (x,y)
   * @return (x, y)
   */
  public double[] getPosition(){
    return position.getCurrentPosition();
  }


  /**
   * Sets the current robot position to the given x, y
   * @param x
   * @param y
   */
  public void setPosition(double x, double y){
    position.setCurrentPosition(x, y);
  }

  /**
   * Returns the current robot heading, normalized.
   * @return
   */
  public double getHeading(){
    return heading.getHeading();
  }
  
  public void setHeading (double heading){
    this.heading.setHeading(heading);
  }
}
