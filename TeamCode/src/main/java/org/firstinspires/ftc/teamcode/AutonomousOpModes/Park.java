package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Park extends AutoSuperOp{
    ElapsedTime time;
    @Override
    public void init() {
        super.init();
        time = new ElapsedTime();
    }

    @Override 
    public void loop() {
        drive.driveRobotCentric(0, -0.5, 0);
        // once robot drives for >= 2.5 secs, it parks
        if (time.seconds() >= 2.5) {
            requestOpModeStop();
        }
    }
}
