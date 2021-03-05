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
        telemetry.addData("vertical position", vertical.getPosition());
        telemetry.addData("front hook position", frontHook.getPosition());
        // calls all methods
        drive();
        intake();
        shooter();
        wobbleGoals();
        stop();
        telemetry.update();
    }
}