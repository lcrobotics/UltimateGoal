package org.firstinspires.ftc.teamcode.AutonomousOpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModes.AutoSuperOp;

@Autonomous
public class ParkAuto extends AutoSuperOp {

    @Override
    public void init() {
        super.init();
    }

    @Override 
    public void loop() {
        drive.driveRobotCentric(0, -0.5, 0);
        // once robot drives for >= 2.5 secs, it parks over shooting line
        if (time.milliseconds() >= 2500) {
            requestOpModeStop();
        }
    }
}
