package org.firstinspires.ftc.teamcode.hardwareMaps;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import java.util.HashMap;
import java.util.ArrayList;


public class BulkDataManager {
  
  HashMap<ExpansionHubEx, RevBulkData> bulkReads;
  
  
  public BulkDataManager(HashMap<ExpansionHubEx, RevBulkData> bulkReads){
    this.bulkReads = new HashMap<>(bulkReads);
  }
  
  
  public void updateBulkData(){
    for(ExpansionHubEx m : bulkReads.keySet()){
      m.setValue(m.getBulkInputData());
    }
  }
  
  
  public ArrayList<Integer> getMotorVelocities(ExpansionHubEx hub, List<ExpansionHubMotor> motors){
    ArrayList<Integer> velocities = new ArrayList<>();
    RevBulkData data = bulkReads.get(hub);
    for(ExpansionHubMotor m : motors){
      velocities.add(data.getMotorVelocity(m));
    }
  }
}
