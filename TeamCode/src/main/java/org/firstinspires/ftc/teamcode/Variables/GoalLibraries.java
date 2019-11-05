package org.firstinspires.ftc.teamcode.Variables;



import static java.lang.Math.abs;

/** This class contains the goal libraries for each possible route of the Autonomous code (with mecanum).
 *
 * The goal libraries are named separately, according to their corresponding location and general task on the
 * field. The goal libraries are labeled as follows, in the order {field side, task}.
 *
 * Red side, building zone, moving foundation: goalLibraryRBF
 * Red side, loading zone, getting skystone: goalLibraryRLS
 * Blue side, building zone, moving foundation: goalLibraryBBF
 * Blue side, loading zone, getting skystone: goalLibraryBLS
 */

public class GoalLibraries {

    //0 is a position change in format {0, x, y}
    //1 is an angle change in format {1, angle, 0}
    //2 is the vision test for skystone in format {2, 1 for red or 2 for blue, 0}
    //3 is an intake cycle
    //4 is a lift change up
    //5 is a lift change down
    //6 is a wait method with the seconds as the second element
    //others as needed


    public double[][] goalLibraryRBF = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryRLS = {{0, 0, 0}, {6, 10000, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryBBF = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] goalLibraryBLS = {{0, 0, 0}, {6, 10000, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    public double[][] choosenLibrary;


    /** Sets the goal library to the chosen goal.
     *
     *
     * @param color Either "red" or "blue"
     * @param task Either "skystone" or "foundation"
     * @throws IllegalArgumentException If you spell those things wrong, it's your fault.
     */
    public GoalLibraries(String color, String task) throws NoFoundGoalLibraryException {


        if(color.equals("blue")){
            if(task.equals("skystone")){
                for(int i = 0; i < goalLibraryBLS.length; i++){
                    for(int j = 0; i < goalLibraryBLS[i].length; j++){
                        choosenLibrary[i][j] = goalLibraryBLS[i][j];
                    }
                }
            }
            else if(task.equals("foundation")){
                for(int i = 0; i < goalLibraryBBF.length; i++){
                    for(int j = 0; i < goalLibraryBBF[i].length; j++){
                        choosenLibrary[i][j] = goalLibraryBBF[i][j];
                    }
                }
            }
            else{
                throw new NoFoundGoalLibraryException(task);
            }

        }
        else if(color.equals("red")){
            if(task.equals("skystone")){
                for(int i = 0; i < goalLibraryRLS.length; i++){
                    for(int j = 0; j < goalLibraryRLS[i].length; j++){
                        choosenLibrary [i][j] = goalLibraryRLS[i][j];
                    }
                }
            }
            else if(task.equals("foundation")){
                for(int i = 0; i < goalLibraryRBF.length; i++) {
                    for (int j = 0; j < goalLibraryRBF[i].length; j++) {
                        choosenLibrary[i][j] = goalLibraryRBF[i][j];
                    }
                }
            }
            else {
                throw new NoFoundGoalLibraryException(task);
            }

        }
        else {
            throw new NoFoundGoalLibraryException(color);
        }

    }



}
