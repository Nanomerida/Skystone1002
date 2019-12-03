package org.firstinspires.ftc.teamcode.Methods;


import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Supplier;

public class GamepadWrapper {

    Gamepad gamepad;

    public GamepadWrapper(Gamepad gamepad){
        this.gamepad = gamepad;
    }


    public boolean a() {
        return gamepad.a;
    }



}
