package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TurnTest extends AutoSuperOpNew {

    boolean ran = false;
    public void loop() {
        if (!ran) {
            time.reset();
            ran = true;
        }


        if (time.milliseconds() > 900) {
            drive.driveRobotCentric(0,0,0);
        } else {
            drive.driveRobotCentric(0, 0, .5);
        }
    }
}
