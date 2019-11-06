package org.firstinspires.ftc.teamcode.Variables;

import android.util.SparseArray;


public class GoalLibrariesHashMap {


    public GoalLibrariesHashMap(){

    }



    private SparseArray<double[]> goalLibraryRBF = new SparseArray<>();

    private SparseArray<double[]> goalLibraryRLS = new SparseArray<>();

    private SparseArray<double[]> goalLibraryBBF = new SparseArray<>();

    private SparseArray<double[]> goalLibraryBLS = new SparseArray<>();

    private SparseArray<double[]> choosenLibrary;


    public SparseArray<double[]> loadLibrary(String library) {
        switch (library) {

            case "goalLibraryRBF":
                goalLibraryRBF.append(0, new double[]{0, 0, 0});
                goalLibraryRBF.append(1, new double[]{0, 0, 0});
                goalLibraryRBF.append(2, new double[]{0, 0, 0});
                goalLibraryRBF.append(3, new double[]{0, 0, 0});
                goalLibraryRBF.append(4, new double[]{0, 0, 0});
                goalLibraryRBF.append(5, new double[]{0, 0, 0});
                goalLibraryRBF.append(6, new double[]{0, 0, 0});
                goalLibraryRBF.append(7, new double[]{0, 0, 0});
                goalLibraryRBF.append(8, new double[]{0, 0, 0});
                goalLibraryRBF.append(9, new double[]{0, 0, 0});
                choosenLibrary = goalLibraryRBF.clone();

            break;
            case "goalLibraryRLS":
                goalLibraryRLS.append(0, new double[]{0, 0, 0});
                goalLibraryRLS.append(1, new double[]{6, 10000, 0});
                goalLibraryRLS.append(2, new double[]{0, 0, 0});
                goalLibraryRLS.append(3, new double[]{0, 0, 0});
                goalLibraryRLS.append(4, new double[]{0, 0, 0});
                goalLibraryRLS.append(5, new double[]{0, 0, 0});
                goalLibraryRLS.append(6, new double[]{0, 0, 0});
                goalLibraryRLS.append(7, new double[]{0, 0, 0});
                goalLibraryRLS.append(8, new double[]{0, 0, 0});
                goalLibraryRLS.append(9, new double[]{0, 0, 0});
                choosenLibrary = goalLibraryRLS.clone();

            break;
            case "goalLibraryBBF":
                goalLibraryBBF.append(0, new double[]{0, 0, 0});
                goalLibraryBBF.append(1, new double[]{0, 0, 0});
                goalLibraryBBF.append(2, new double[]{0, 0, 0});
                goalLibraryBBF.append(3, new double[]{0, 0, 0});
                goalLibraryBBF.append(4, new double[]{0, 0, 0});
                goalLibraryBBF.append(5, new double[]{0, 0, 0});
                goalLibraryBBF.append(6, new double[]{0, 0, 0});
                goalLibraryBBF.append(7, new double[]{0, 0, 0});
                goalLibraryBBF.append(8, new double[]{0, 0, 0});
                goalLibraryBBF.append(9, new double[]{0, 0, 0});
                choosenLibrary = goalLibraryBBF.clone();

            break;
            case "goalLibraryBLS":
                goalLibraryBLS.append(0, new double[]{0, 0, 0});
                goalLibraryBLS.append(1, new double[]{6, 10000, 0});
                goalLibraryBLS.append(2, new double[]{0, 0, 0});
                goalLibraryBLS.append(3, new double[]{0, 0, 0});
                goalLibraryBLS.append(4, new double[]{0, 0, 0});
                goalLibraryBLS.append(5, new double[]{0, 0, 0});
                goalLibraryBLS.append(6, new double[]{0, 0, 0});
                goalLibraryBLS.append(7, new double[]{0, 0, 0});
                goalLibraryBLS.append(8, new double[]{0, 0, 0});
                goalLibraryBLS.append(9, new double[]{0, 0, 0});
                choosenLibrary = goalLibraryBLS.clone();

            break;
        }

        return choosenLibrary;
    }


}
