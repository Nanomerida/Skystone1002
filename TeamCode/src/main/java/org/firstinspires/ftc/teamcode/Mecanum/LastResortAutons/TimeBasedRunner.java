package org.firstinspires.ftc.teamcode.Mecanum.LastResortAutons;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.google.common.collect.MultimapBuilder;
import com.google.common.collect.ArrayListMultimap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class TimeBasedRunner {
  
  private static ArrayList<Action> actionList = new ArrayList<>();
  private static HashMap<Action, Long> actionDelays = new HashMap<>();
  private long startTime;
  private static ElapsedTime timer;
  private static long actionNum;
  private LinearOpMode opMode;
  
  public TimeBasedRunner(long startTime, LinearOpMode opMode){
    this.startTime = startTime;
    actionNum = 0;
    this.opMode = opMode;
  }
  
  public void addAction(Action newAction, long delayFromLast){
    actionList.add(actionNum, newAction);
    actionDelays.put(newAction, delayFromLast);
    actionNum++;
  }
  
  public void startRunning(){
      actionNum = 0;
      timer = new ElapsedTime(startTime);
      Action currentAction = actionList.get(actionNum);
      long delay = actionDelays.get(currentAction);
      while(opMode.opModeIsActive()){
        if(!(timer.milliseconds() => delay)){
          currentAction.invoke();
        }
      }
  }




}
