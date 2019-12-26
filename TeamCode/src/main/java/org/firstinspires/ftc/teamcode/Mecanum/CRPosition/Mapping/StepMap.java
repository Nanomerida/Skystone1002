package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Mapping;

import android.util.SparseArray;


public class StepMap {

    SparseArray<Step> stepMap;

    int currentStep = 0;

    public StepMap(){
        this.stepMap = new SparseArray<>();
    }

    public StepMap(int initialCapacity){
        this.stepMap = new SparseArray<>(initialCapacity);
    }

    public  void addStep(int key, Step step){
        stepMap.put(key, step);
    }

    public Step currentStep(){
        return stepMap.get(currentStep);
    }

    public boolean onPositionStep(){
        Step stepNow = stepMap.get(currentStep);
        if(stepNow.getClass() == PositionStep.class){
            return true;
        }
        else {
            return false;
        }
    }

    public boolean onAngleStep(){
        Step stepNow = stepMap.get(currentStep);
        if(stepNow.getClass() == AngleStep.class){
            return true;
        }
        else {
            return false;
        }
    }

    public double[] getCurrentStep(){
        return stepMap.get(currentStep).getAsArray();
    }

    public double[] nextStep(){
        currentStep++;
        return stepMap.get(currentStep).getAsArray();
    }

    public void goToNextStep(){
        currentStep++;
    }

    public double[] getStep(int key){
        return stepMap.get(key).getAsArray();
    }



}
