package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {


    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    public DcMotor lift_motor = null;





    //Crates HardwareMap object robot

    public float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }

    //Initializes with the hardwareMap
    @Override
    public void init() {

        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    public void loop() {

        if (gamepad2.right_bumper) {
            lift_motor.setPower(0.5);
        }
        else if (gamepad2.left_bumper) {
            lift_motor.setPower(-0.5);
        } else {
            lift_motor.setPower(0);
        }


        float[] inputs = {gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x};
        float[] outputs;

        float[] forward = {1, 1, 1, 1};
        float[] right = {-1, 1, -1, 1};
        float[] c_turn = {-1, -1, 1, 1};

        float[][] matrix = {forward, right, c_turn};

        outputs = m_v_mult(matrix, inputs);

        left_front_drive.setPower(outputs[3]);
        left_back_drive.setPower(outputs[2]);
        right_front_drive.setPower(outputs[0]);
        right_back_drive.setPower(outputs[1]);






    }
}