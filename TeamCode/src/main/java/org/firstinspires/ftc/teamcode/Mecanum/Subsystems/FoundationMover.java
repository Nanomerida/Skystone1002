package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FoundationMover implements Subsystem {


    private CRServo left = null;
    private CRServo right = null;


    public FoundationMover(HardwareMap hardwareMap){
        left = hardwareMap.get(CRServo.class, "left_mover");
        right = hardwareMap.get(CRServo.class, "right_mover");
    }

    @Override
    public void init(){
        right.setDirection(CRServo.Direction.REVERSE);
    }



    public void down(){
        byPower(-0.2);
    }

    public void up(){
        byPower(0.2);
    }

    public void stop(){
        byPower(0);
    }

    public void byPower(double power){
        left.setPower(power);
        right.setPower(power);
    }
}
