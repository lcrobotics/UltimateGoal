package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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