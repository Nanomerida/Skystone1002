package org.firstinspires.ftc.teamcode.hardware;

public final class DriveBaseVectors {


    public static final float[] forward = {1.0f, 1.0f, 1.0f, 1.0f};

    public static final float[] backward = {-0.75f, -1.0f, -0.7f, -1.0f};

    public static final float[] strafeL = {-0.8f, 0.95f, 0.85f, -0.95f};

    public static final float[] strafeR = {0.90f, -0.35f, -0.90f, 0.35f};
/**Nelitha's edit:*/
    //public static final float[] strafeR = {0.8f, -0.95f, -0.95f, 0.95f};

    public static final float[] turnCW = {0.75f, 1.0f, -0.75f, -1.0f};

    public static final float[] turnCCW = {-0.75f, -1.0f, 0.75f, 1.0f};

    public static final float[][] arcadeDriveVectors = {forward, strafeR, turnCW};
}
