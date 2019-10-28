package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardwareMaps.HardwareMapMain;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {
    //Crates HardwareMap object robot
    HardwareMapMain robot = new HardwareMapMain();

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
        robot.init(hardwareMap);

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

        robot.left_front_drive.setPower(outputs[2]);
        robot.left_back_drive.setPower(outputs[3]);
        robot.right_front_drive.setPower(outputs[0]);
        robot.right_back_drive.setPower(outputs[1]);






    }
}