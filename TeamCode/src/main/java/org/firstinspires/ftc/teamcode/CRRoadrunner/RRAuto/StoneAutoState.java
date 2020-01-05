package org.firstinspires.ftc.teamcode.CRRoadrunner.RRAuto;

import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;

/*
.addMarker(2.0,() -> {servo.setPosition(1); return Unit.INSTANCE;})
 */

public enum StoneAutoState {

    DRIVING_TO_SENSING_POS,

    SENSING,

    SKYSTONE_LEFT,

    SKYSTONE_RIGHT,

    SKYSTONE_UNSEEN,

    AT_SKYSTONE,

    HAVE_SKYSTONE,

    DRIVING_TO_LOADING_ZONE,

    AT_LOADING_ZONE,

    DROPPED_SKYSTONE,

    GOING_BACK,

    AT_STONE,

    HAVE_STONE,

    /**
     * Use the AT_LOADING_ZONE case and the
     * DROPPED_SKYSTONE case here
     */
    GOING_BACK_TO_LOADING_ZONE,

    PARKING,

    FINISHED;

    private StoneAutoState secondSkystone;

    public void storeSecondSkystonePos(Vuforia.SkystonePosition firstPos){
        switch (firstPos){
            case LEFT: secondSkystone = StoneAutoState.SKYSTONE_LEFT;
            case RIGHT: secondSkystone = StoneAutoState.SKYSTONE_RIGHT;
            case UNSEEN: secondSkystone = StoneAutoState.SKYSTONE_UNSEEN;
        }
    }

    public StoneAutoState getSecondSkystonePos(){
        return secondSkystone;
    }






}
