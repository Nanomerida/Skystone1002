package org.firstinspires.ftc.teamcode.Variables;



    public class NoFoundGoalLibraryException extends Exception{

        String term;
        public NoFoundGoalLibraryException(String aterm){
            this.term = aterm;
        }

        public String toString(){
            return (term + "is not a valid choice!");
        }
    }





