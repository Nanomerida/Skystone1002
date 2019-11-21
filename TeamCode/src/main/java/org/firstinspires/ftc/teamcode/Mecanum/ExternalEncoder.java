package org.firstinspires.ftc.teamcode.Mecanum;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ExternalEncoder {
  
  DcMotor encoder;
  int totalCounts;
  int deltaCounts;
  
  
  public ExternalEncoder(DcMotor encoder) {
    this.encoder = encoder;
  }
  
  public void syncEncoders(){
    totalCounts = encoder.getCurrentPosition();
  }  
  



}
