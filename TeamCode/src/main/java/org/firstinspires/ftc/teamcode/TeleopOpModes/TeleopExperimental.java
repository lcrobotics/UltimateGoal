package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleopOpModes.SuperOp;

@TeleOp
public class TeleopExperimental extends SuperOpExperimental {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        telemetry.addData("frontLeft Mult", frontLeftDrive.multiplier);
        telemetry.addData("frontRight Mult", frontRightDrive.multiplier);
        telemetry.addData("backLeft Mult", backLeftDrive.multiplier);
        telemetry.addData("backRight Mult", backRightDrive.multiplier);

        drive();
        intake();
        shooter();
        wobbleGoals();
        stop();
    }
}