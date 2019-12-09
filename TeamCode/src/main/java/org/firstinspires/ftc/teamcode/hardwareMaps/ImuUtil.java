package org.firstinspires.ftc.teamcode.hardwareMaps;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Duo motor rtp test", group = "Testing")

public class ImuUtil extends LinearOpMode {
    
    DcMotor lift = null;
    
    int counts;


    @Override
    public void runOpMode(){
        
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0);
        
        waitForstart();
        
        while(opModeIsActive()){
            counts = lift.getCurrentPosition();
            
            if(gamepad1.dpad_up) {
                counts += 1;
                lift.setTargetPosition(counts);
                lift.setPower(.5);
            } 
            else if(gamepad.dpad_down){
                counts -= 1;
                lift.setTargetPosition(counts);
                lift.setPower(.5);
            }
            else {
                lift.setTargetPosition(counts);
                lift.setPower(0);
            }
            
        }
        lift.setPower(0);
        
        
    }    

    
    
 }
    
    
      
