package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Competition Auto")
public class FirstAuto extends AutoSuperOp {
    // start the OpMode in state DRIVE
    AutoState auto = AutoState.DRIVE;
    @Override
    public void init() {
        super.init();
    }

    public void loop() {
        switch (auto) {
            // drive from starting position to just past shooting line, allowing camera to see target
            case DRIVE:
                // make sure this only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // begin to drive forward, towards shooting line
                drive.driveRobotCentric(0, -0.5, 0);
                // when time > 3, reset lock, stop robot, and go to state STOPCHECK
                if (time.seconds() >= 3) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.STOPCHECK;
                }

                break;
            // stop robot and get location, then use as a springboard to get to other states
            case STOPCHECK:
                // add telemetry for rotNum value (so we know how many times the robot has turned)
                telemetry.addData("rotNum", rotNum);

                // make sure code only runs worse
                if (!lock) {
                    drive.stop();
                    time.reset();
                    lock = true;
                }

                // if time is between .1 and .3 check for location of robot and proceed into state movement
                // if time > .5, proceed to turning states by conditionals
                if (time.seconds() <= 0.3 && time.seconds() >= 0.1) {

                    // attempt to get robot's location based on nav target
                    objectLocator.updateRobotLocation();
                    // add telemetry telling us if robot can see target
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    // strafe angle is only true if the code has been to state SIDEWAYS, requiring
                    // the robot to be at about 90 degrees, facing the target
                    // if it is true, update position and change to state ANGLE
                    if (strafeAngle) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        auto = AutoState.ANGLE;
                        break;
                    }

                    // if not trying to find target, update position
                    if (checkState > 0) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                    }

                    // if adjusting angle, proceed to state ANGLE (checkState == 1)
                    // if strafing, proceed to state SIDEWAYS (checkState == 2)
                    if (checkState == 1) {
                        auto = AutoState.ANGLE;
                        break;
                    } else if (checkState == 2) {
                        auto = AutoState.SIDEWAYS;
                        break;
                    }

                    // if target located, update position, reset rotation, and switch to state ANGLE
                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        rotNum = 0;
                        auto = AutoState.ANGLE;
                    }

                } else if (time.seconds() > 0.5) {
                    lock = false;

                    // if rotNum == 2, 3, 4, 5, switch to state COUNTERCLOCKWISE
                    // if rotNum == 8, switch to state FAIL (code is over)
                    // if rotNum == 1, 6, 7, switch to state CLOCKWISE
                    if (rotNum >= 2 && rotNum <= 5) {
                        auto = AutoState.COUNTERCLOCKWISE;
                    } else if (rotNum == 8) {
                        auto = AutoState.FAIL;
                        rotNum = 0;
                        break;
                    } else {
                        auto = AutoState.CLOCKWISE;
                    }
                    // increment rotNum
                    rotNum++;
                }

                break;

            // turn robot clockwise slowly - for attempting to find nav targets
            case CLOCKWISE:

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, 0.2);

                // if time > 1, switch state to STOPCHECK (this both switches the state and stops the robot)
                if (time.seconds() > 1) {
                    lock = false;
                    auto = AutoState.STOPCHECK;
                }

                break;

            // turn robot counter clockwise slowly - used if can't find nav targets while turning clockwise
            case COUNTERCLOCKWISE:

                // mkae sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counter clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, -0.2);

                // if time > 1, switch state to STOPCHECK (this both switches the state and stops the robot)
                if (time.seconds() > 1) {
                    lock = false;
                    auto = AutoState.STOPCHECK;
                }

                break;

            // get the robot's position based on the nav target and correct the angle until the robot
            // is facing the nav target head on
            case ANGLE:

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if time > .1, switch state to STOPCHECK
                // if checkState == 0, set it equal to 1
                if (time.seconds() > 0.1) {
                    lock = false;
                    if (checkState == 0) {
                        checkState = 1;
                    }
                    auto = AutoState.STOPCHECK;
                }

                // find optimal angle for the robot to face
                objectLocator.updateRobotLocation();
                // update position of the robot
                lastPos = objectLocator.lastPos;
                // add telemetry for the angle of the robot (in relation to nav target)
                telemetry.addData("angle", lastPos.w);


                // adjust position until angle is within pre-decided threshold
                if (lastPos.w > 91) {
                    drive.driveRobotCentric(0, 0, -0.3);
                } else if (lastPos.w < 89) {
                    drive.driveRobotCentric(0, 0, 0.3);
                // if angle is within threshold, switch to state SIDEWAYS
                } else {
                    // stop driving
                    drive.stop();
                    lock = false;

                    // if strafeAngle == true, switch to state SIDEWAYS
                    if (strafeAngle) {
                        auto = AutoState.SIDEWAYS;
                    }
                    // if adjusting the angle the first time (rot == 0), switch to state SIDEWAYS
                    if (rot == 0) {
                        auto = AutoState.SIDEWAYS;
                    } else {
                        // if neither condition is true, set rot = 0 and switch to state SHOOT
                        rot = 0;
                        auto = AutoState.SHOOT;
                    }
                }

                break;

            // finds translational delta (only R/L) and adjusts so that the robot is within a
            // pre-determined threshold
            case SIDEWAYS:
                // set strafeAngle = true (so that the states with conditionals requiring it can
                // know that the angle is close enough)
                strafeAngle = true;

                // declare desiredY position (eg: about where the robot so be in the y direction on the field)
                // NOTE: the Y is actually horizontal, due to rev
                double desiredY = 33;

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // adjust position until the robot is within a pre-decided threshold
                if (lastPos.y > desiredY + 3) {
                    drive.driveRobotCentric(0.4, 0, 0);
                } else if (lastPos.y < desiredY - 3) {
                    drive.driveRobotCentric(-0.4, 0, 0);
                // when robot within threshold, reset variables, stop driving, and switch to state BACK
                } else {
                    strafeAngle = false;
                    drive.stop();
                    lock = false;
                    auto = AutoState.BACK;
                    break;
                }

                // if time > .5 switch to state STOPCHECK
                if (time.seconds() > 0.5) {
                    lock = false;

                    // if checkstate == 1, set to checkState = 2;
                    if (checkState == 1) {
                        checkState = 2;
                    }
                    auto = AutoState.STOPCHECK;
                }

                // add telemetry for y position
                telemetry.addData("y", lastPos.y);

                break;

            // find distance the robot needs to move behind the shooting line
            case BACK:

                // declare desiredX position (eg: about where the robot so be in the x direction on the field)
                // NOTE: the X is actually vertical, due to rev
                double desiredX = 44;

                // find optimal angle for the robot to face
                objectLocator.updateRobotLocation();
                // update position of the robot
                lastPos = objectLocator.lastPos;

                // add telemetry for the angle of the robot (in relation to nav target)
                telemetry.addData("x", lastPos.x);

                // adjust robot position based on pre-determined values
                if (lastPos.x > desiredX + 1) {
                    drive.driveRobotCentric(0, 0.3, 0);
                } else if (lastPos.x < desiredX - 1) {
                    drive.driveRobotCentric(0, -0.3, 0);
                // if robot is the correct position, switch to state BACK2
                } else {
                    drive.stop();
                    auto = AutoState.BACK2;
                }

                break;

            // drive to behind shooting line using information from state BACK
            case BACK2:

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive to behind shooting line
                drive.driveRobotCentric(0, 0.3, 0);

                // if time > 3, reset variables, stop driving, and switch to state ANGLE
                if (time.seconds() >= 3) {
                    lock = false;
                    drive.stop();
                    rot++;
                    auto = AutoState.ANGLE;
                }

                break;

            // shoot ring into mid or high goal (depending on how shitty the motor is being)
            case SHOOT:

                // make sure code only runs once, set shooter to on, shooter runs until state is switched
                if (!lock) {
                    shooter.set(1);
                    time.reset();
                    lock = true;
                    target = true;
                }

                // if time > .5, begin toggling shooterServo
                // NOTE: the shooter must be given enough time to get to full power, hence the wait time
                if (time.seconds() > 0.5) {
                    // set shooterServo = 0, second half of shooting (eg: it closes)
                    shooterServo.setPosition(target? 0 : 1);
                    time.reset();
                    // toggle target
                    target = !target;
                    // increment ringsShot
                    ringsShot++;
                } else {
                    // set servo to 1 = first half of shooting (eg: it opens)
                    shooterServo.setPosition(target? 1 : 0);
                }

                // caps the number of shots at 3 (ringsShot keeps track of how many times shooterServo
                // has opened/closed, so the actual rings shot will be half of the value
                // if ringsShot == 6, switch to state PARK
                if (ringsShot == 6) {
                    lock = false;
                    auto = AutoState.PARK;
                }

                break;
            // park over shooting line
            case PARK:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if time > 1.8, drive to end up over shooting line
                if (time.seconds() >= 1.8) {
                    drive.driveRobotCentric(0, -0.3, 0);
                }

                // if time >= 2.5, stop robot and switch to state FAIL
                if (time.seconds() >= 2.5) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.FAIL;
                }

                break;

            // stops OpMode, either after state PARK or in the case of catastrophic failure
            case FAIL:
                // stop the entire OpMode, not just motors
                requestOpModeStop();

                break;
        }
    }
}