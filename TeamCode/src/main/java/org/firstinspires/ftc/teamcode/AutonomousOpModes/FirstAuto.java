package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Competition Auto")
public class FirstAuto extends AutoSuperOp {
    // what state of autonomous we are in
    AutoState auto = AutoState.DRIVE;
    // number of attempts to find nav target
    int rotNum = 0;
    ObjectLocator.RobotPos lastPos;
    ElapsedTime time;
    boolean target;
    boolean strafeAngle = false;
    boolean lock = false;
    int ringsShot = 0;
    int rot = 0; // 0 when adjusting angle the first time, 1 when adjusting angle the second time
    int checkState = 0; // 0 when checking for target during rotation, 1 when angle adjusting, 2 when strafing, 3 when going back
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

                drive.driveRobotCentric(0, -0.5, 0);
                // once robot drives for >3 secs, goes to clockwise case
                // resets lock
                if (time.seconds() >= 3) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.STOPCHECK;
                }
                break;

            case STOPCHECK:

                telemetry.addData("rotNum", rotNum);

                if (!lock) {
                    drive.stop();
                    time.reset();
                    lock = true;
                }

                if (time.seconds() <= 0.3 && time.seconds() >= 0.1) {

                    // attempt to get robot's location based on nav target
                    objectLocator.updateRobotLocation();
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    if (strafeAngle) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        auto = AutoState.ANGLE;
                        break;
                    }

                    if (checkState > 0) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                    }
                    if (checkState == 1) {
                        auto = AutoState.ANGLE;
                        break;
                    } else if (checkState == 2) {
                        auto = AutoState.SIDEWAYS;
                        break;
                    }

                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        rotNum = 0;
                        auto = AutoState.ANGLE;
                    }

                } else if (time.seconds() > 0.5) {
                    lock = false;

                    if (rotNum >= 2 && rotNum <= 5) {
                        auto = AutoState.COUNTERCLOCKWISE;
                    } else if (rotNum == 8) {
                        auto = AutoState.FAIL;
                        rotNum = 0;
                        break;

                    } else {
                        auto = AutoState.CLOCKWISE;
                    }
                    rotNum++;
                }

                break;

                // Turns robot clockwise to look for picture.
            case CLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, 0.2);

                if (time.seconds() > 1) {
                    lock = false;
                    auto = AutoState.STOPCHECK;
                }

                break;

                // if robot passes picture, turns the other way (counterclockwise) to look again.
            case COUNTERCLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, -0.2);

                if (time.seconds() > 1) {
                    lock = false;
                    auto = AutoState.STOPCHECK;
                }

                break;

                // get robot's position based on nav target and update angle
            // until the robot is facing the nav target directly
            case ANGLE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                if (time.seconds() > 0.1) {
                    lock = false;
                    if (checkState == 0) {
                        checkState = 1;
                    }
                    auto = AutoState.STOPCHECK;
                }

                // optimal angle the robot should be facing
                objectLocator.updateRobotLocation();
                lastPos = objectLocator.lastPos;

                telemetry.addData("angle", lastPos.w);

                // adjust position until angle is within pre-decided threshold
                if (lastPos.w > 91) {
                    drive.driveRobotCentric(0, 0, -0.3);
                } else if (lastPos.w < 89) {
                    drive.driveRobotCentric(0, 0, 0.3);
                // if angle is within threshold, go to sideways case
                } else {
                    drive.stop();
                    lock = false;
                    if (strafeAngle) {
                        auto = AutoState.SIDEWAYS;
                    }
                    if (rot == 0) {
                        auto = AutoState.SIDEWAYS;
                    } else {
                        rot = 0;
                        auto = AutoState.SHOOT;
                    }
                }

                break;

                // finds translational delta (only R/L) and adjusts to within threshold.
            case SIDEWAYS:

                strafeAngle = true;

                // get robot's position and update sideways position
                double desiredY = 33;

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // adjust position until robot pos is within a pre-decided threshold
                if (lastPos.y > desiredY + 3) {
                    drive.driveRobotCentric(0.4, 0, 0);
                } else if (lastPos.y < desiredY - 3) {
                    drive.driveRobotCentric(-0.4, 0, 0);
                    // when robot within threshold, go to back case
                } else {
                    strafeAngle = false;
                    drive.stop();
                    lock = false;
                    auto = AutoState.BACK;
                    break;
                }

                // 0.2
                if (time.seconds() > 0.5) {
                    lock = false;
                    if (checkState == 1) {
                        checkState = 2;
                    }
                    auto = AutoState.STOPCHECK;
                }

                telemetry.addData("y", lastPos.y);

                break;
                // finds distance robot needs to go to be behind shooting line and adjusts.
            case BACK:

                // get robot's position and go back
                double desiredX = 44;

                objectLocator.updateRobotLocation();
                lastPos = objectLocator.lastPos;

                telemetry.addData("x", lastPos.x);

                // adjust robot position based on pre-determined values
                if (lastPos.x > desiredX + 1) {
                    drive.driveRobotCentric(0, 0.3, 0);
                } else if (lastPos.x < desiredX - 1) {
                    drive.driveRobotCentric(0, -0.3, 0);
                // if robot in right place, go to shoot case
                } else {
                    drive.stop();
                    auto = AutoState.BACK2;
                }
                break;

            case BACK2:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0.3, 0);

                // resets lock
                if (time.seconds() >= 3) {
                    lock = false;
                    drive.stop();
                    rot++;
                    auto = AutoState.ANGLE;
                }
                break;

                // shoots ring into goal
            case SHOOT:

                if (!lock) {
                    // sets shooter motor to 1, will run for rest of case
                    shooter.set(1);
                    time.reset();
                    lock = true;
                    target = true;
                }


                if (time.seconds() > 0.5) {

                    // set servo to 0 = second half of shooting action
                    shooterServo.setPosition(target? 0 : 1);
                    time.reset();
                    target = !target;
                    ringsShot++;

                } else {
                    // set servo to 1 = first half of shooting action
                    shooterServo.setPosition(target? 1 : 0);
                }
                // caps the number of shots at 3
                if (ringsShot == 6) {
                    lock = false;
                    auto = AutoState.PARK;
                }
                break;

            case PARK:
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                if (time.seconds() >= 1.8) {
                    drive.driveRobotCentric(0, -0.3, 0);
                }

                if (time.seconds() >= 2.5) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.FAIL;
                }
                break;

                // stops OpMode
            case FAIL:
                requestOpModeStop();
                break;
        }
    }
}