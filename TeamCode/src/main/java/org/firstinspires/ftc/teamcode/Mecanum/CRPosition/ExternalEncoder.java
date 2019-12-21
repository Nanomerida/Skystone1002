package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class ExternalEncoder {

  ExpansionHubMotor encoder;
  private static double convRate = 0.006184246955207152778;
  private int previousCounts;
  private int deltaCounts;
  private RevBulkData bulkData;

  public ExternalEncoder(ExpansionHubMotor encoder, RevBulkData bulkData) {
    this.encoder = encoder;
    this.bulkData = bulkData;
    this.encoder.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void reverseEncoder(){
    this.encoder.setDirection(ExpansionHubMotor.Direction.REVERSE);
  }


  public void syncEncoders() {
    previousCounts = bulkData.getMotorCurrentPosition(encoder);
    deltaCounts = 0;
  }

  public int getCounts() {
    deltaCounts = (bulkData.getMotorCurrentPosition(encoder) - previousCounts);
    return deltaCounts;
  }

  public double getInches(){
    return getCounts() * convRate;
  }

  public void hardReset(){
    encoder.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
  }


}
