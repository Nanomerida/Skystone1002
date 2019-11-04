package org.firstinspires.ftc.teamcode.Methods;

import java.util.HashMap;


/** This class simply contains the HashMap @link(https://www.w3schools.com/java/java_hashmap.asp)_
 * for the mecanum moving procedures. It also contains the method that is used to use the HashMap in
 * other classes. -Ryan D */


public class MecMoveProcedureStorage {




    //The HashMap with the Mecanum movement procedures.


    private HashMap<String, float[]> mecanum = new HashMap<String, float[]>();

    //Constructor
    public MecMoveProcedureStorage(){
        mecanum.put("forward", new float[] {0.5f, 0.5f, 0.5f, 0.5f}); //forward
        mecanum.put("backward", new float[] {-0.5f, -0.5f, -0.5f, -0.5f} ); //backward
        mecanum.put("strafeL", new float[] {-0.5f, 0.5f, 0.5f, -0.5f}); //strafe left
        mecanum.put("strafeR", new float[] {0.5f, -0.5f, -0.5f, 0.5f}); //strafe right
        mecanum.put("rotateCC", new float[] {0.5f, 0.5f, -0.5f, -0.5f}); //rotate clockwise
        mecanum.put("rotateCCW", new float[] {-0.5f, -0.5f, 0.5f, 0.5f}); //rotate counter-clockwise

    }





    //The method that is called in other classes to use the HashMap
    public HashMap<String, float[]> getMecanum() {
        return mecanum;
    }

}
