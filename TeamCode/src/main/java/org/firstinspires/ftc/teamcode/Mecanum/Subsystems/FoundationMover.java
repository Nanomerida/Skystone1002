package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class FoundationMover implements Subsystem {

    
    private Servo left = null;
    private Servo right = null;


    public FoundationMover(HardwareMap hardwareMap){
        left = hardwareMap.get(Servo.class, "left_foundation_mover");
        right = hardwareMap.get(Servo.class, "right_foundation_mover");
    }

    @Override
    public void init(){
        //You can now set the position as if it was the opposite way.
        right.setDirection(Servo.Direction.REVERSE);
        down();
    }



    public void down(){
        left.setPosition(0);
        right.setPosition(0);
    }

    public void up(){
        left.setPosition(1);
        right.setPosition(1);
    }

    public void close(){
        left.setPosition(0.2);
        right.setPosition(0.2);
    }
    
    

    public void byPower(double power){
        left.setPower(power);
        right.setPower(power);
    }

    public Function0<Unit> getDown(){
        return () -> {
            down();
            return Unit.INSTANCE;
        };
    }

    public Function0<Unit> getUp(){
        return () -> {
            up();
            return Unit.INSTANCE;
        };
    }

}
