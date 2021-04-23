package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Competition TeleOp")
public class Teleop extends SuperOp {
    @Override
    public void loop() {
        drive();
        intake();
        shooter();
        wobbleGoals();
        driverStop();
    }
}