package org.firstinspires.ftc.teamcode.CRRoadrunner.RRAuto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class BlueFoundationAuto extends  LinearOpMode{

    public static FoundationAutoState autoState;

    private SampleMecanumDriveBase drive;

    @Override
    public void runOpMode(){

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);


    }
}
