package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopNew extends SuperOpNew{
    @Override
    public void init() {
        super.init();
    }

    public void loop() {
        // call TeleOp methods
        drive();
        intake();
        index();
        shooter();
        wobbleGoals();
        emergencyStop();
    }
}
