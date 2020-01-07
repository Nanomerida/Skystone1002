package org.firstinspires.ftc.teamcode.CRRoadrunner.RRAuto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.CRRoadrunner.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.CRVuforia.Vuforia;
import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.FoundationMover;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

@Autonomous
public class BlueStoneAuto extends LinearOpMode {


    /**From center of robot
     *
     */
    private static final Pose2d startingPosition = new Pose2d(0, 0, 0);

    private SampleMecanumDriveBase drive;
    private Vuforia vuforia = new Vuforia();
    private FoundationMover foundationMover;

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


    /**
     * As far as I can tell, the coordinate system is like this
     *
     * x-positive = building zone
     * x-negative = loading zone
     * y-positive = blue side
     * y-negative = red side
     */
    @Override
    public void runOpMode(){




        /*Set up the drive class

         */
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        /*Setup the foundation movers

         */
        foundationMover = new FoundationMover(hardwareMap);

        /*Init vuforia

         */
        vuforia.blueInit(hardwareMap, this);

        /*
        Define starting position here
         */
        drive.setPoseEstimate(startingPosition);

        waitForStart();

        if(!isStopRequested()) return;

        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(drive.getPoseEstimate().getY() + 10)
                        .build()
        );



    }
}
