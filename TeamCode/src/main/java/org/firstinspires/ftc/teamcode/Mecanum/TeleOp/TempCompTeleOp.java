package org.firstinspires.ftc.teamcode.Mecanum.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;


@TeleOp(name = "Pls work")
@Disabled
public class TempCompTeleOp extends LinearOpMode {



    public ExpansionHubMotor left_front_drive = null;
    public ExpansionHubMotor left_back_drive = null;
    public ExpansionHubMotor right_front_drive = null;
    public ExpansionHubMotor right_back_drive = null;
    public Servo claw = null;
    public CRServo arm = null;
    public ExpansionHubMotor lift_left = null;
    public ExpansionHubMotor lift_right = null;


    float[] inputs;
    float[] outputs;
    private float[][] matrix = {{0.55f, 1.0f, 0.5f, 1.0f}, {0.7f, -0.95f, -0.75f, 0.95f}, {0.65f, 1.0f, -0.65f, -1.0f}};



    private static float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }

    @Override
    public void runOpMode(){

        left_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "left_back_drive");
        right_front_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "right_back_drive");

        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.crservo.get("arm");

        lift_left =  (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "lift_left");
        lift_right =  (ExpansionHubMotor)hardwareMap.get(DcMotor.class, "lift_right");


        left_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);


        right_front_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);
        right_back_drive.setDirection(ExpansionHubMotor.Direction.REVERSE);


        lift_left.setDirection(DcMotor.Direction.REVERSE);

        lift_left.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        lift_right.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            inputs = new float[] {gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x};
            outputs = m_v_mult(matrix, inputs);
            left_front_drive.setPower(outputs[0]);
            left_back_drive.setPower(outputs[1]);
            right_front_drive.setPower(outputs[2]);
            right_back_drive.setPower(outputs[3]);

            if(gamepad2.right_bumper) claw.setPosition(0);
            else if(gamepad2.left_bumper) claw.setPosition(0.4);

            arm.setPower(-gamepad2.left_stick_y);

            if(gamepad2.dpad_up) {lift_left.setPower(-0.5); lift_right.setPower(-0.5); }
            else if(gamepad2.dpad_down) {lift_left.setPower(0.1); lift_right.setPower(0.1); }
            else lift_left.setPower(0); lift_right.setPower(0);
        }




    }
}
