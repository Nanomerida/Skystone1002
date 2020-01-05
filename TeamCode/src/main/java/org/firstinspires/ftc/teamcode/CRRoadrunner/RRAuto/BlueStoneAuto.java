package org.firstinspires.ftc.teamcode.CRRoadrunner.RRAuto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

@Autonomous
public class BlueStoneAuto extends LinearOpMode {

    public static StoneAutoState autoState;

    private SampleMecanumDriveBase drive;

    /*
    .addMarker(2.0,() -> {servo.setPosition(1); return Unit.INSTANCE;})
     */



    Trajectory toSensingPos;

    Trajectory toSkystoneLeft;

    Trajectory toSkystoneRight;

    Trajectory toSkystoneUnseen;

    Trajectory toLoadingZone;

    Trajectory backToBuildingZone;

    Trajectory toSecondStoneLeft;

    Trajectory toSecondStoneRight;

    Trajectory toSecondStoneUnseen;

    Trajectory toParkingSpot;


    @Override
    public void runOpMode(){




        drive = new SampleMecanumDriveREVOptimized(hardwareMap);


    }
}
