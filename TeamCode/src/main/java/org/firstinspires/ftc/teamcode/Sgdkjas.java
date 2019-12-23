package org.firstinspires.ftc.teamcode;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;


public class Sgdkjas {


    public static void main(String[] args){
        try{
            Scanner user = new Scanner(System.in);
            System.out.println("Enter file name:");

            String filename = user.nextLine();
            File file = new File(filename);
            Scanner reader = new Scanner(file);

            File myObj = new File(filename);
            if (myObj.exists()) {
                System.out.println("File name: " + myObj.getName());
                System.out.println("Absolute path: " + myObj.getAbsolutePath());
                System.out.println("Writeable: " + myObj.canWrite());
                System.out.println("Readable " + myObj.canRead());
                System.out.println("File size in bytes " + myObj.length());
            } else {
                System.out.println("The file does not exist.");
            }
            

            while(reader.hasNextLine()){
                System.out.println(reader.nextLine());
            }

            reader.close();
            user.close();

        }catch(Exception e){
            System.out.println("OOPOPOPOP");
            e.printStackTrace();
        }


    }
}