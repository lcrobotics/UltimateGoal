package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SuperOp;

public class Autonomous extends SuperOp {

    final int DRIVE = 0;
    final int CLOCKWISE = 1;
    final int DELTA = 2;
    final int COUNTERCLOCKWISE = 3;
    final int SHOOT = 4;

    int auto = DRIVE;

    ElapsedTime time;
    double start;

    boolean lock = false;
    ElapsedTime lockTime;
    double lockStart = lockTime.milliseconds();

    boolean picture;

    @Override
    public void init() {
        super.init();
    }

    public void loop() {

        switch (auto) {

            case DRIVE:

                if (lock = false){
                    start = time.milliseconds();
                    lock = true;
                }

                driveForward.driveRobotCentric(0, 0.5, 0, true);
                if (time.milliseconds() - start >= 3000) {
                    lock = false;
                    auto = CLOCKWISE;
                }
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
                if (lock = false) {
                    if (lockTime.milliseconds() - lockStart >= 1000) {
                        lock = true;
                    } else {
                        driveForward.driveRobotCentric(0, 0.5, 0, true);
                    }
                }

                if (lock = false) {
                    if (lockTime.milliseconds() - lockStart >= 1000) {
                        lock = true;
                    } else {
                        driveForward.driveRobotCentric(0.5, 0, 0, true);
                    }
                }

                //find rotational delta
                if (lock = false) {
                    if (lockTime.milliseconds() - lockStart >= 1000) {
                        lock = true;
                    } else {
                        driveForward.driveRobotCentric(0, 0, 0.5, true);
                    }
                }

                //if deltas in threshold:
                if (lock = false) {
                    if (lockTime.milliseconds() - lockStart >= 3000) {
                        lock = true;
                    } else {
                        driveForward.driveRobotCentric(0, -0.5, 0, true);
                    }
                }

                lock = false;
                auto = SHOOT;

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