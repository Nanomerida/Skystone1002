package org.firstinspires.ftc.teamcode.Mecanum.Config;

public class RobotDimensions {


    public static double trackWidth = 14.72071;

    public static final double X_ENCODER_OFFSET = 0.686417;
    public static final double DISTANCE_TO_TOP_RIGHT_X = 8.267717;
    public static final double DISTANCE_TO_TOP_RIGHT_Y = 8.724134;




    public enum RobotCorner {
        TOP_RIGHT,
        TOP_LEFT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT;

        public double[] translateToRobotCenter(double[] pre) {
            double[] post = new double[2];
            switch (this) {
                case TOP_LEFT:
                    post[0] = pre[0] + DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = pre[1] - DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case TOP_RIGHT:
                    post[0] = pre[0] - DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = pre[1] - DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case BOTTOM_LEFT:
                    post[0] = pre[0] + DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = pre[1] + DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case BOTTOM_RIGHT:
                    post[0] = pre[0] - DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = pre[1] + DISTANCE_TO_TOP_RIGHT_Y;
                    break;
            }
        return post;
        }
        public double[] translateToRobotCenter(double x, double y) {
            double[] post = new double[2];
            switch (this) {
                case TOP_LEFT:
                    post[0] = x + DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = y - DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case TOP_RIGHT:
                    post[0] = x - DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = y - DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case BOTTOM_LEFT:
                    post[0] = x + DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = y + DISTANCE_TO_TOP_RIGHT_Y;
                    break;
                case BOTTOM_RIGHT:
                    post[0] = x - DISTANCE_TO_TOP_RIGHT_X;
                    post[1] = y + DISTANCE_TO_TOP_RIGHT_Y;
                    break;
            }
            return post;
        }
    }


}
