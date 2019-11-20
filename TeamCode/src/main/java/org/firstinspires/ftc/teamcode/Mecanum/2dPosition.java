package org.firstinspires.ftc.teamcode.Mecanum;

import org.firstinspires.ftc.external.navigation.Positon;
import org.firstinspires.ftc.external.navigation.DistanceUnit;


public class 2dPosition {

  double x;
  double y;
  final long timeStamp;

  public 2dPosition(double x, double y) {
    this.timeStamp = System.nanoTime();
    this.x = x;
    this.y = y;
  }

  public double[] getCurrentPosition(){
    return new double[] {x, y};
  }
  
  public long getTimeStam() {
    return timeStamp;
  }



}
