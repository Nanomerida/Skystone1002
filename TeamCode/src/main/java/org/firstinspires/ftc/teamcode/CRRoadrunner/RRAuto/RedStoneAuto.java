package org.firstinspires.ftc.teamcode.CRRoadrunner.RRAuto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;

import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.FoundationMover;


@Autonomous
public class RedStoneAuto extends LinearOpMode {
    /**Form center of robot!!
     *
     */

    private static final Pose2d startingPosition = new Pose2d(0, 0, -90);

    public static StoneAutoState autoState;

    private SampleMecanumDriveBase drive;
    private Vuforia vuforia = new Vuforia();
    private FoundationMover foundationMover;




    @Override
    public void runOpMode(){

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        foundationMover = new FoundationMover(hardwareMap);

        vuforia.blueInit(hardwareMap, this);

        drive.setPoseEstimate(startingPosition);

        waitForStart();

        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(drive.getPoseEstimate().getY() + 10)
                        .build()
        );







    }

}
