package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends SuperOp {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        // calls all methods
        wobbleGoals();
        shooter();
        intake();
        stop();
        drive();
    }
}