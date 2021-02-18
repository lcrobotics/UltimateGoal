package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SuperOp;

@Autonomous
public class FirstAuto extends SuperOp {

    public enum AutoState {
        DRIVE(0),
        CLOCKWISE(1),
        ANGLE(2),
        SIDEWAYS(3),
        BACK(4),
        COUNTERCLOCKWISE(5),
        SHOOT(6),
        FAIL(7);

        int val;

        AutoState(int value) {
            val = value;
        }
    }

    // what state of autonomous we are in
    AutoState auto = AutoState.DRIVE;
    // number of attempts to find nav target
    int rotNum = 0;
    ObjectLocator.RobotPos lastPos;
    ElapsedTime time;
    boolean target;
    boolean lock = false;
    int ringsShot = 0;
    @Override
    public void init() {
        super.init();
        time = new ElapsedTime();
    }

    public void loop() {

        switch (auto) {

            // drive from starting position to just past shooting line
            // so that the sensors can see the picture.
            case DRIVE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0.5, 0);
                if (time.seconds() >= 3) {
                    lock = false;
                    auto = AutoState.CLOCKWISE;
                }
                break;

                // Turns robot clockwise to look for picture.
            case CLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // select time for rotation based on the rotNum we are on
                int rotTime = 4;
                if (rotNum == 0 || rotNum == 4) {
                    rotTime = 2;
                }

                drive.driveRobotCentric(0, 0, 0.5);

                // attempt to get robot's location based on nav target
                objectLocator.updateRobotLocation();
                if (objectLocator.targetVisible) {
                    lock = false;
                    auto = AutoState.ANGLE;
                } else if (time.seconds() >= rotTime) {
                    lock = false;
                    if (rotNum == 4) {
                        auto = AutoState.FAIL;
                    } else {
                        auto = AutoState.COUNTERCLOCKWISE;
                        rotNum++;
                    }
                }
                break;

                // if robot passes picture, turns the other way (counterclockwise) to look again.
            case COUNTERCLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, -0.5);

                objectLocator.updateRobotLocation();
                if (objectLocator.targetVisible) {
                    lock = false;
                    auto = AutoState.ANGLE;
                } else if (time.seconds() >= 4) {
                    lock = false;
                    rotNum++;
                    auto = AutoState.CLOCKWISE;
                }
                break;

                // get robot's position based on nav target and update angle until the robot is facing the nav target directly
            case ANGLE:

                lastPos = objectLocator.lastPos;

                if (lastPos.w > 92) {
                    drive.driveRobotCentric(0, 0, 0.5);
                } else if (lastPos.w < 88) {
                    drive.driveRobotCentric(0, 0, -0.5);
                } else {
                    auto = AutoState.SIDEWAYS;
                }

                break;

                // finds translational delta (only R/L) and adjusts to within threshold.
            case SIDEWAYS:

                // get robot's position and update sideways position
                double desiredY = 300;

                lastPos = objectLocator.lastPos;

                if (lastPos.y > desiredY + 2) {
                    drive.driveRobotCentric(0.5, 0, 0);
                } else if (lastPos.y < desiredY - 2) {
                    drive.driveRobotCentric(-0.5, 0, 0);
                } else {
                    auto = AutoState.BACK;
                }

                // finds distance robot needs to go to be behind shooting line and adjusts.
            case BACK:

                // get robot's position and go back
                double desiredX = 500;

                lastPos = objectLocator.lastPos;

                if (lastPos.x > desiredX + 2) {
                    drive.driveRobotCentric(0, -0.5, 0);
                } else if (lastPos.x < desiredX - 2) {
                    drive.driveRobotCentric(0, 0.5, 0);
                } else {
                    auto = AutoState.SHOOT;
                }

                // shoots ring into goal
            case SHOOT:

                if (!lock) {
                    shooter.set(1);
                    time.reset();
                    lock = true;
                }

                shooterServo.setPosition(target? 1 : 0);

                if (time.seconds() > 0.5) {
                    shooterServo.setPosition(target? 0 : 1);
                    time.reset();
                    ringsShot++;
                }
                if (ringsShot == 3) {
                    lock = false;
                    auto = AutoState.FAIL;
                }
                break;

            case FAIL:
                requestOpModeStop();
                break;
        }
    }
}