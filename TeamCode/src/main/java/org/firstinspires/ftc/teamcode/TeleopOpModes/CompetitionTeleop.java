package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CompetitionTeleop extends SuperOp {
    @Override
    public void init() {
        super.init();
    }

    public void loop() {
        // call TeleOp methods
        drive();
        intake();
        shooterRewrite(); // manually active
        wobbleGoals();
        emergencyStop();
    }
}
