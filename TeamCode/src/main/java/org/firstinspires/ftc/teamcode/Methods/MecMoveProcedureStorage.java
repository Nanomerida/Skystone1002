package org.firstinspires.ftc.teamcode.Methods;

import java.util.HashMap;


/** This class simply contains the HashMap @link(https://www.w3schools.com/java/java_hashmap.asp)_
 * for the mecanum moving procedures. It also contains the method that is used to use the HashMap in
 * other classes. -Ryan D */


public class MecMoveProcedureStorage {




    //The HashMap with the Mecanum movement procedures.


    private HashMap<String, int[]> mecanum = new HashMap<String, int[]>();

    //Constructor
    public MecMoveProcedureStorage(){
        mecanum.put("forward", new int[] {1, 1, 1, 1}); //forward
        mecanum.put("backward", new int[] {-1, -1, -1, -1} ); //backward
        mecanum.put("strafeL", new int[] {-1, 1, 1, -1}); //strafe left
        mecanum.put("strafeR", new int[] {1, -1, -1, 1}); //strafe right
        mecanum.put("rotateCC", new int[] {1, 1, -1, -1}); //rotate clockwise
        mecanum.put("rotateCCW", new int[] {-1, -1, 1, 1}); //rotate counter-clockwise

    }





    //The method that is called in other classes to use the HashMap
    public HashMap<String, int[]> getMecanum() {
        return mecanum;
    }

}
