package org.firstinspires.ftc.teamcode.Mecanum;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class ExternalEncoder {
  
  ExpansionHubMotor encoder;
  private int previousCounts;
  private int deltaCounts;
  
  
  public ExternalEncoder(ExpansionHubMotor encoder) {
    this.encoder = encoder;
  }
  
  
  public void syncEncoders(RevBulkData bulkData){
    previousCounts = bulkData.getMotorCurrentPosition(encoder);
    deltaCounts = 0;
  }
  
  public int getCounts(RevBulkData bulkData){ 
    deltaCounts = (bulkData.getMotorCurrentPosition(encoder) - previousCounts);
    return deltaCounts;
  }
  



}
