package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Park extends AutoSuperOp{

    @Override
    public void init() {
        super.init();
    }

    @Override 
    public void loop() {
        drive.driveRobotCentric(0, -0.5, 0);
        // once robot drives for >= 2.5 secs, it parks over shooting line
        if (time.seconds() >= 2.5) {
            requestOpModeStop();
        }
    }
}
