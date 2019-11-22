package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class ExternalEncoder {

  ExpansionHubMotor encoder;
  private int previousCounts;
  private int deltaCounts;


  public ExternalEncoder(ExpansionHubMotor encoder) {
    this.encoder = encoder;
  }


  public void syncEncoders(RevBulkData bulkData) {
    previousCounts = bulkData.getMotorCurrentPosition(encoder);
    deltaCounts = 0;
  }

  public int getCounts(RevBulkData bulkData) {
    deltaCounts = (bulkData.getMotorCurrentPosition(encoder) - previousCounts);
    return deltaCounts;
  }

  public void hardReset(){
    encoder.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
    encoder.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
  }


}
