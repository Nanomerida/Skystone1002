package org.firstinspires.ftc.teamcode.CRTelemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;


public class TeleOpTelem {



    private Telemetry telemetry;
    private ArrayList<ExpansionHubMotor> driveMotors;
    private ArrayList<CRServo> foundationMovers;
    private ArrayList<DigitalChannel> limitSwitches;
    private ExpansionHubEx expansionHubEx1;
    private ExpansionHubEx expansionHubEx10;
    private RevBulkData revBulkData1;

    //Our telemetry dashboard
    //The items are public so they can be edited outside of this class, for debugging purposes
    //and so this class isn't too long/

    //Drive motor velocities; we don't include speed because bulk reads can only get velocity.
    public Telemetry.Item leftFrontVelo;
    public Telemetry.Item leftBackVelo;
    public Telemetry.Item rightFrontVelo;
    public Telemetry.Item rightBackVelo;

    //The foundation moving servos
    public Telemetry.Item leftFoundation;
    public Telemetry.Item rightFoundation;


    //The limit switches

    //Bottom
    public Telemetry.Item leftBottomSwitch;
    public Telemetry.Item rightBottomSwitch;

    //Top
    public Telemetry.Item leftTopSwitch;
    public Telemetry.Item rightTopSwitch;

    //The expansion hub(s) current status;
    public Telemetry.Item expansionHub1;
    public Telemetry.Item expansionHub10;




    public TeleOpTelem(Telemetry telemetry, ArrayList<ExpansionHubMotor> driveMotors, ArrayList<CRServo> foundationMovers,
                      ArrayList<DigitalChannel> limitSwitches, ExpansionHubEx expansionHubEx1, ExpansionHubEx expansionHubEx10, RevBulkData revBulkData1){
        this.telemetry = telemetry;
        this.driveMotors = driveMotors;
        this.foundationMovers = foundationMovers;
        this.limitSwitches = limitSwitches;
        this.expansionHubEx1 = expansionHubEx1;
        this.expansionHubEx10 = expansionHubEx10;
        this.revBulkData1 = revBulkData1;
    }


    /**
     * Initializes all the telemetry objects and sets them not to clear when {@link Telemetry#update()}
     * is called.
     */
    public void setUpTelemetry(){



        leftFrontVelo = telemetry.addData("Left Front Motor Velocity", 0);
        leftBackVelo = telemetry.addData("Left Back Motor Velocity", 0);
        rightFrontVelo = telemetry.addData("Right Front Motor Velocity", 0);
        rightBackVelo = telemetry.addData("Right Back Motor Velocity", 0);



        leftFoundation = telemetry.addData("Left Foundation Mover Power", "3%.2f", foundationMovers.get(0).getPower());
        rightFoundation = telemetry.addData("Right Foundation Mover Power", "3%.2f", foundationMovers.get(1).getPower());

        leftBottomSwitch = telemetry.addData("Left Bottom Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(0)));
        rightBottomSwitch = telemetry.addData("Right Bottom Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(1)));
        leftTopSwitch = telemetry.addData("Left Top Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(2)));
        rightTopSwitch = telemetry.addData("Right Top Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(3)));

        expansionHub1 = telemetry.addData("Expansion Hub 1 Over Temp", expansionHubEx1.isModuleOverTemp());
        expansionHub10 = telemetry.addData("Expansion HUb 10 Over Temp", expansionHubEx10.isModuleOverTemp());

        leftFrontVelo.setRetained(true);
        leftBackVelo.setRetained(true);
        rightFrontVelo.setRetained(true);
        rightBackVelo.setRetained(true);
        leftFoundation.setRetained(true);
        rightFoundation.setRetained(true);
        leftBottomSwitch.setRetained(true);
        rightBottomSwitch.setRetained(true);
        leftTopSwitch.setRetained(true);
        rightTopSwitch.setRetained(true);
        expansionHub1.setRetained(true);
        expansionHub10.setRetained(true);

        telemetry.update();

    }

    /**
     * Updates the telemetry screen using only values that can be gotten from
     * the bulk data reads. This makes it much less costly than {@link #fullUpdate()}
     */
    public void quickUpdate(){
        leftFrontVelo.setValue(revBulkData1.getMotorVelocity(driveMotors.get(0)));
        leftBackVelo.setValue(revBulkData1.getMotorVelocity(driveMotors.get(1)));
        rightFrontVelo.setValue(revBulkData1.getMotorVelocity(driveMotors.get(2)));
        rightBackVelo.setValue(revBulkData1.getMotorVelocity(driveMotors.get(3)));

        leftBottomSwitch = telemetry.addData("Left Bottom Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(0)));
        rightBottomSwitch = telemetry.addData("Right Bottom Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(1)));
        leftTopSwitch = telemetry.addData("Left Top Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(2)));
        rightTopSwitch = telemetry.addData("Right Top Limit Switch", revBulkData1.getDigitalInputState(limitSwitches.get(3)));

        telemetry.update();

    }

    /**
     * Updates the telemetry screen with all values. This is slightly more costly than {@link #quickUpdate()},
     * but has more data.
     */

    public void fullUpdate(){
        quickUpdate();

        leftFoundation.setValue("3%.2f", foundationMovers.get(0).getPower());
        rightFoundation.setValue("3%.2f", foundationMovers.get(1).getPower());

        expansionHub1.setValue( expansionHubEx1.isModuleOverTemp());
        expansionHub10.setValue(expansionHubEx10.isModuleOverTemp());

        telemetry.update();
    }

    /**Sends an urgent message to the telemetry board.
     *
     * <b>WARNING!! This will clear <i>all</i> telemetry!</b>
     *
     * Uses include loss of control, or fatal error messages( although if the opMode ends
     * after this, the telemetry will be wiped anyway so....... yeah).
     *
     * @param message The message to display
     */
    public void interruptTelem(String message){
        telemetry.clearAll();
        telemetry.addLine(message);
        telemetry.update();
    }

}
