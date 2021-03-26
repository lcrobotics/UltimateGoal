package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleopOpModes.SuperOp;

@TeleOp(name = "Competition TeleOp")
public class Teleop extends SuperOp {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        drive();
        intake();
        shooter();
        wobbleGoals();
        stop();
    }
}