package org.firstinspires.ftc.teamcode.hardware;


import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;

public class BulkDataManager {



    private RevBulkData bulkData;

    private ExpansionHubEx expansionHubEx;


    public BulkDataManager(ExpansionHubEx expansionHubEx1, RevBulkData bulkData1){
        this.expansionHubEx = expansionHubEx1;
        this.bulkData = bulkData1;
    }


    public void refreshBulkData(){
        bulkData = expansionHubEx.getBulkInputData();
    }

    public RevBulkData getBulkData(){
        return bulkData;
    }

    public ExpansionHubEx getHubs(){
        return expansionHubEx;
    }
}
