package org.firstinspires.ftc.teamcode.Mecanum;


public class TwoDOrientation {
  
  
  double heading;
  String unit;
  
  public TwoDOrientation(double heading, String unit) {
    this.heading = heading;
    this.unit = unit;
  }


  public double getHeading(){
    return heading;
  }
  
  public void setHeading(double heading){
    this.heading = heading;
  }
  
  public void setUnit(String unit){
    this.unit = unit;
  }
  
  public String toString() {
    return heading + " " + unit;
  }
}
