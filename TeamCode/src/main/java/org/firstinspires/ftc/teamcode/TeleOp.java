package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOp", group="TeleOp")

public class TeleOp extends OpMode {
    //Crates HardwareMap object robot
    CRGreenHardwareMap robot = new CRGreenHardwareMap(telemetry);

    public float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        out[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0];
        out[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1];
        out[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2];
        out[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3];
        return out;
    }
    DcMotor left_front_drive;
    DcMotor left_back_drive;
    DcMotor right_front_drive;
    DcMotor right_back_drive;

    //Initializes with the hardwareMap
    @Override
    public void init() {
        robot.init(hardwareMap);

        left_front_drive = hardwareMap.dcMotor.get("leftFrontMotor");
        left_back_drive = hardwareMap.dcMotor.get("leftBackMotor");
        right_front_drive = hardwareMap.dcMotor.get("rightFrontMotor");
        right_back_drive = hardwareMap.dcMotor.get("rightBackMotor");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    public void loop() {


        float[] inputs = {gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x};
        float[] outputs;

        float[] forward = {1, 1, 1, 1};
        float[] right = {-1, 1, -1, 1};
        float[] c_turn = {-1, -1, 1, 1};

        float[][] matrix = {forward, right, c_turn};

        outputs = m_v_mult(matrix, inputs);

        left_front_drive.setPower(outputs[2]);
        left_back_drive.setPower(outputs[3]);
        right_front_drive.setPower(outputs[0]);
        right_back_drive.setPower(outputs[1]);






    }
}