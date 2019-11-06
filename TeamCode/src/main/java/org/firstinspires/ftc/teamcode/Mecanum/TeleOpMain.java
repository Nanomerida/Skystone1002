package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Methods.MecMoveProcedureStorage;
import java.util.HashMap;

@TeleOp(name = "TeleOpMain", group="TeleOp")

public class TeleOpMain extends OpMode {

    MecMoveProcedureStorage procedures = new MecMoveProcedureStorage();
    private HashMap<String, float[]> mecanum = procedures.getMecanum();

    public DcMotor left_front_drive = null;
    public DcMotor left_back_drive = null;
    public DcMotor right_front_drive = null;
    public DcMotor right_back_drive = null;
    public DcMotor lift_motor = null;
    public CRServo intake_wheel_left = null;
    public CRServo intake_wheel_right = null;

    public float[] m_v_mult(float[][] m, float[] v) {
        float[] out = new float[4];
        for (int i = 0; i < 4; i++) {
            out[i] = v[i] * m[0][i] + v[1] * m[1][i] + v[2] * m[2][i];
        }
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

        intake_wheel_left = hardwareMap.get(CRServo.class, "intake_wheel_left");
        intake_wheel_right = hardwareMap.get(CRServo.class, "intake_wheel_right");


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        intake_wheel_right.setDirection(CRServo.Direction.REVERSE);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    public void loop() {

        telemetry.addLine("Motor Powers | ")
                .addData("Left Front:", left_front_drive.getPower())
                .addData("Left Back:", left_back_drive.getPower())
                .addData("Right Front:", right_front_drive.getPower())
                .addData("Right Back:", right_back_drive.getPower());

        if (gamepad2.right_bumper) {
            lift_motor.setPower(0.5);
        }
        else if (gamepad2.left_bumper) {
            lift_motor.setPower(-0.5);
        } else {
            lift_motor.setPower(0);
        }

        intake_wheel_left.setPower(gamepad2.right_trigger);
        intake_wheel_right.setPower(gamepad2.right_trigger);

        float[] inputs = {gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x};
        float[] outputs;

        float[] forward = mecanum.get("forward");
        float[] right = mecanum.get("strafeR");
        float[] c_turn = mecanum.get("turnCC");

        float[][] matrix = {forward, right, c_turn};

        outputs = m_v_mult(matrix, inputs);

        left_front_drive.setPower(outputs[3]);
        left_back_drive.setPower(outputs[2]);
        right_front_drive.setPower(outputs[0]);
        right_back_drive.setPower(outputs[1]);

    }
}