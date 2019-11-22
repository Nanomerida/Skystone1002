package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


public class TwoDPosition {

  static double x;
  static double y;
  final long timeStamp;

  public TwoDPosition(double x, double y) {
    this.timeStamp = System.nanoTime();
    this.x = x;
    this.y = y;
  }

  public double[] getCurrentPosition(){
    return new double[] {x, y};
  }
  
  public void setCurrentPosition(double x, double y){
    this.x = x;
    this.y = y;
  }
  
  public long getTimeStam() {
    return timeStamp;
  }



}
