package org.firstinspires.ftc.teamcode.Mecanum.DogeCommanderOps;

import com.disnodeteam.dogecommander.DogeCommander;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.disnodeteam.dogecommander.DogeOpMode;

import org.firstinspires.ftc.teamcode.Mecanum.Subsystems.FoundationMoverSubsystem;

public class BlueCommandBased extends LinearOpMode implements DogeOpMode {

    FoundationMoverSubsystem foundationMover = new FoundationMoverSubsystem(hardwareMap, FoundationMoverSubsystem.CurrentFoundation.DOWN);




    @Override
    public void runOpMode() throws InterruptedException {

        //The doge commander object
        DogeCommander commander = new DogeCommander(this);

        //register subsystems here
        commander.registerSubsystem(foundationMover);

        //Initiate the commander and all subsystems
        commander.init();




        waitForStart();

        commander.start();





    }
}
