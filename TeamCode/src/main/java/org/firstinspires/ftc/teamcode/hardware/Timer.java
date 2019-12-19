package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {

    ElapsedTime time;
    long stopTime;
    long current;

    public Timer(){
        this.time = new ElapsedTime();
    }


    public void setTimer(long millis){
        this.stopTime = millis;
    }

    public void startTimer(){
        time.reset();
    }

    public void pauseTimer(){
        current =(long)time.milliseconds();
    }

    public void resumeTimer(){
        time = new ElapsedTime(current);
    }

    public boolean timerDone(){
        return (time.milliseconds() == stopTime);
    }

}
