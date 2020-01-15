package org.firstinspires.ftc.teamcode.Mecanum.Subsystems;


import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class FoundationMoverSubsystem implements Subsystem {

    public enum CurrentFoundation {

        DOWN(0),
        HALF_DOWN(0.2),
        UP(1);

        private final double positionVal;

        CurrentFoundation(double positionVal){
            this.positionVal = positionVal;
        }
    }


    private Servo left = null;
    private Servo right = null;

    private HardwareMap hardwareMap;

    private CurrentFoundation state;




    public FoundationMoverSubsystem(HardwareMap hardwareMap, CurrentFoundation startState){

        this.state = startState;
        this.hardwareMap = hardwareMap;

    }

    @Override
    public void initHardware(){
        left = hardwareMap.get(Servo.class, "left_mover");
        right = hardwareMap.get(Servo.class, "right_mover");

        //You can now set the position as if it was the opposite way.
        right.setDirection(Servo.Direction.REVERSE);

    }



    @Override
    public void periodic(){

        setCurrentPos();
    }

    public void setPos(double pos){
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void setCurrentPos(){
        setState(state);
    }


    public void setState(CurrentFoundation newState){
        setPos(newState.positionVal);
        state = newState;
    }

    public CurrentFoundation getState(){
        return state;
    }



    public void down(){
        setState(CurrentFoundation.DOWN);
    }

    public void up(){
        setState(CurrentFoundation.UP);
    }

    public void close(){
        setState(CurrentFoundation.HALF_DOWN);
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
