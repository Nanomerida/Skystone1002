package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


public class TwoDOrientation {
  
  
  static double Currentheading;
  String unit;
  
  public TwoDOrientation(double heading, String unit) {
    Currentheading = heading;
    this.unit = unit;
  }


  public double getHeading(){
    return Currentheading;
  }
  
  public void setHeading(double heading){
    Currentheading = heading;
  }
  
  public void setUnit(String unit){
    this.unit = unit;
  }
  
  public String toString() {
    return Currentheading + " " + unit;
  }
}
