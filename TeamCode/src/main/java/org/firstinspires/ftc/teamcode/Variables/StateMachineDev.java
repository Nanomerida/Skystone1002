package org.firstinspires.ftc.teamcode.Variables;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CRVuforia.VuforiaBlue;
import org.firstinspires.ftc.teamcode.Methods.GeneralMethods;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;



/**This development class is for the blue skystone task */

@Autonomous(name="StateMachine", group="Testing")
public class StateMachineDev extends OpMode{

  private GeneralMethods methods = new GeneralMethods();
  private ElapsedTime timer = new ElapsedTime(0);
  private VuforiaBlue blockPosBlue = new VuforiaBlue(); //creates an instance of the vuforia blue side file

  public enum RobotStates {
    WAIT_FOR_TIME,
    DRIVE_TO_STONES,
    DRIVE_ROTATE,
    VISION_TEST,
    INTAKE_CYCLE,
    LIFT_UP,
    LIFT_DOWN;

    RobotStates(){
    }

  }


  public DcMotor left_front_drive   = null;
  public DcMotor  left_back_drive  = null;
  public DcMotor  right_front_drive = null;
  public DcMotor  right_back_drive = null;
  public DcMotor  left_y_encoder = null;
  public DcMotor  right_y_encoder = null;
  public DcMotor  x_encoder = null;
  public DcMotor  lift = null;
  public CRServo intake_wheel_left = null;
  public CRServo intake_wheel_right = null;
  public WebcamName webcam = null;

  public StateMachineDev() {

  }

  public BNO055IMU imu;
  private Orientation angles;

  private double[] AbsolutePosition(double PrevX, double PrevY) {
    double[] PreviousPosition = {PrevX, PrevY};
    double[] CurrentPosition = new double[2];
    double[] XEncoderPosition = new double[2];
    double[] YEncoderPosition = new double[2];
    double ConvRate = (PI * 90) / 208076.8;
    double[] WeirdOrlandoMathsX = {sin(degreesConversion()), cos(degreesConversion())};
    double[] WeirdOrlandoMathsY = {cos(degreesConversion()), sin(degreesConversion())};
    int XClicks = ticksX();
    int YClicks = ((ticksLeftY() + ticksRightY()) / 2);
    for(int i = 0; i < 2; i++) {
      XEncoderPosition[i] = ConvRate * XClicks * WeirdOrlandoMathsX[i];
      YEncoderPosition[i] = ConvRate * YClicks * WeirdOrlandoMathsY[i];
    }
    for(int k = 0; k < 2; k++) {
      CurrentPosition[k] = XEncoderPosition[k] + YEncoderPosition[k] + PreviousPosition[k];
    }
    resetEncoders();
    return CurrentPosition;
  }
  private int ticksLeftY(){
    int deltaTicks;
    deltaTicks = (previousTicksYLeft - left_y_encoder.getCurrentPosition());
    return deltaTicks;
  }

  private int ticksRightY(){
    int deltaTicks;
    deltaTicks = (previousTicksYRight - right_y_encoder.getCurrentPosition());
    return deltaTicks;
  }

  private int ticksX(){
    int deltaTicks;
    deltaTicks = (previousTicksX - x_encoder.getCurrentPosition());
    return deltaTicks;
  }

  private void resetEncoders(){
    previousTicksYLeft = left_y_encoder.getCurrentPosition();
    previousTicksYRight = right_y_encoder.getCurrentPosition();
    previousTicksX = x_encoder.getCurrentPosition();
  }

  private double degreesConversion(){
    double theta = this.angles.firstAngle;
    if(theta < 0) theta += 360;
    if(redSide){
      if(theta > 0) theta -= 180;
      else theta += 180;
    }
    return theta;
  }

  private void moveDrivebyPower(double[] powers) { //method to move.
    int x = 0;
    for (DcMotor motor : driveMotors) {
      motor.setPower(powers[x]);
      x++;
    }
  }

  private void MovePosition(){
    //Use odometry to move
        moveDrivebyPower(methods.PositionChange(actualPos[0], goalLibrary[stepNumber][1], actualPos[2], goalLibrary[stepNumber][2]));
      previousPos[0] = actualPos[0];
      previousPos[1] = actualPos[1];
  }

  private void stopDrive(){
    for(DcMotor motor : driveMotors){
      motor.setPower(0);
    }
  }



  private void newState(RobotStates newState) {
    // Reset the state time, and then change to next state.
    //StateTime.reset();
    currentState = newState;
  }



  RobotStates currentState;
  private static int stepNumber = 1;
  private static double[] previousPos;
  private static int stonePos;
  private static int previousTicksYLeft = 0;
  private static int previousTicksYRight = 0;
  private static int previousTicksX = 0;
  private static boolean redSide;
  private static double actualPos[];

  private ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>();

  private double[][] goalLibrary;

  @Override
  public void init(){



    try {
      GoalLibraries library = new GoalLibraries("blue", "skystone");
      goalLibrary = library.choosenLibrary;
    }
    //Runs a scuffed opMode if no goal library can be loaded
    catch(NoFoundGoalLibraryException e) {

      telemetry.addData("Well,", "We @^#*&@^ed up.");
      telemetry.update();

    }
    previousPos[0] = goalLibrary[0][0]; //sets the starting position for the robot
    previousPos[1] = goalLibrary[0][1];
    if(goalLibrary[0][2] == 1) redSide = true;




    //Initialize motors
    left_front_drive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    left_back_drive = hardwareMap.get(DcMotor.class, "leftBackDrive");
    right_front_drive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    right_back_drive = hardwareMap.get(DcMotor.class, "rightBackDrive");

    //Lift
    lift = hardwareMap.get(DcMotor.class, "slide_motor");

    //External encoders
    left_y_encoder = hardwareMap.get(DcMotor.class, "left_y_encoder");
    right_y_encoder = hardwareMap.get(DcMotor.class, "right_y_encoder");
    x_encoder = hardwareMap.get(DcMotor.class, "x_encoder");

    //Intake wheels
    intake_wheel_left = hardwareMap.get(CRServo.class, "intake_wheel_left");
    intake_wheel_right = hardwareMap.get(CRServo.class, "intake_wheel_right");

    //Webcam
    webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    //Initialize vuforia with webcam
    blockPosBlue.blueInit(webcam);

    // MORE IMU STUFF

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


    currentState = RobotStates.WAIT_FOR_TIME;

  }

  @Override
  public void init_loop(){

  }

  @Override
  public void start(){

  }

  @Override
  public void loop(){

    timer.reset();

    // Execute the current state.  Each STATE's case code does the following:
    // 1: Look for an EVENT that will cause a STATE change
    // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
    //   else
    // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.

    switch (currentState){

      case WAIT_FOR_TIME:
        if(timer.seconds() == 10){
          newState(RobotStates.DRIVE_TO_STONES);
        }
        else {
          telemetry.update();
        }
        break;


      case DRIVE_TO_STONES:
        actualPos = AbsolutePosition(previousPos[0], previousPos[1]);
        if(methods.GoalCheckPos(actualPos[0], goalLibrary[stepNumber][1], actualPos[1], goalLibrary[stepNumber][2])){
            newState(RobotStates.VISION_TEST);
        }
        else{
          telemetry.update();

        }
        break;

      case VISION_TEST:
        break;



    }

  }

  @Override
  public void stop(){

  }






}