package org.firstinspires.ftc.teamcode.Variables;



import static java.lang.Math.cos; //Ryan's Math Stuff
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

/** This class contains the goal libraries for each possible route of the Autonomous code (with mecanum).
 *
 * The goal libraries are named separately, according to their corresponding location and general task on the
 * field. The goal libraries are labeled as follows, in the order {field side, task}.
 *
 * Red side, building zone, getting skystone: goalLibraryRBS
 * Red side, loading zone, getting skystone: goalLibraryRLS
 * Blue side, building zone, moving foundation: goalLibraryBBF
 * Blue side, loading zone, getting skystone: goalLibraryBLS
 */

public class GoalLibraries {


    public GoalLibraries(){

    }

    //0 is a position change in format {0, x, y}
    //1 is an angle change in format {1, angle, 0}
    //2 is the vision test for skystone in format {2, 1 for red or 2 for blue, 0}
    //3 is an arm change in format {3, arm power, claw head servo}
    //4 is an claw change in format {4, open(0)/close(1) claw, 0}
    //5 is a slide change.
    //others as needed


    public double[][] goalLibraryRBS = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
            {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryRLS = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
            {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryBBF = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
            {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryBLS = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
            {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};



}
