package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;

import org.firstinspires.ftc.teamcode.SuperOp;

public class Autonomous extends SuperOp {

    int auto = 0;

    final int DRIVE = 0;
    final int CLOCKWISE = 1;
    final int DELTA = 2;
    final int COUNTERCLOCKWISE = 3;
    final int SHOOT = 4;

    boolean picture;

    @Override
    public void init() {
        super.init();
    }

    public void loop() {

        switch (auto) {

            case DRIVE:
                driveForward.driveRobotCentric(0, 0.5, 0, true);
                auto = CLOCKWISE;
                break;

            case CLOCKWISE:
                driveForward.driveRobotCentric(0, 0, 0.5, true);

                if (picture) {
                    auto = DELTA;
                } else {
                    auto = COUNTERCLOCKWISE;
                }
                break;

            case DELTA:
                //find translational delta

                //find rotational delta

                //if deltas in threshold, go to SHOOT

                //if not, go to DELTA
                break;

            case COUNTERCLOCKWISE:
                driveForward.driveRobotCentric(0, 0, -0.5, true);

                if (picture) {
                    auto = DELTA;
                } else {
                    auto = CLOCKWISE;
                }
                break;

            case SHOOT:
                //shoot
                break;
        }
    }

    public boolean pictureFound() {
            return false;
    }
}