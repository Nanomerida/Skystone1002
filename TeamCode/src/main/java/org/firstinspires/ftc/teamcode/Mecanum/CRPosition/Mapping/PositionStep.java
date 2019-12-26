package org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Mapping;

import org.firstinspires.ftc.teamcode.Mecanum.CRPosition.Vector2d;

public class PositionStep implements Step {


    Vector2d position;
    long delay;

    public PositionStep(Vector2d position, long delay){
        this.position = new Vector2d(position);
        this.delay = delay;
    }

    public PositionStep(double[] position, long delay){
        this.position = new Vector2d(position[0], position[1]);
        this.delay = delay;
    }

    public PositionStep(double x, double y, long delay){
        this.position = new Vector2d(x, y);
        this.delay = delay;
    }

    public Vector2d getPosition() {
        return position;
    }

    @Override
    public double[] getAsArray(){
        return new double[] {position.getX(), position.getY(), (double) delay};
    }

    public long getDelay(){
        return delay;
    }
}
