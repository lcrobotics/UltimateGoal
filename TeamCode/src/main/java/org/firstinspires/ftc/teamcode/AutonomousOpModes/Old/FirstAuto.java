package org.firstinspires.ftc.teamcode.AutonomousOpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModes.AutoState;
import org.firstinspires.ftc.teamcode.AutonomousOpModes.AutoSuperOp;

@Autonomous
public class FirstAuto extends AutoSuperOp {
    // start the OpMode in state DRIVEOVERMID
    AutoState auto = AutoState.DRIVEOVERMID;

    @Override
    public void init() {
        super.init();
    }

    public void loop() {
        switch (auto) {
            // drive from starting position to just past shooting line, allowing camera to see servoPos
            case DRIVEOVERMID:
                // make sure this only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // begin to drive forward, towards shooting line
                drive.driveRobotCentric(0, -0.5, 0);
                // when time >= 3000 milliseconds, reset lock, stop robot, and go to state UPDATE
                if (time.milliseconds() >= 3000) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.UPDATE;
                }

                break;

            // stop robot and get location, then use as a springboard to get to other states
            case UPDATE:
                // add telemetry for turnCount value (so we know how many times the robot has turned)
                telemetry.addData("turnCount", turnCount);

                // make sure code only runs once, stop motors and reset time
                if (!lock) {
                    drive.stop();
                    time.reset();
                    lock = true;
                }

                // if time is between 100 milliseconds and 300 milliseconds check for location of robot and proceed into state movement
                // if time >= 500 milliseconds, proceed to turning states by conditionals using turnCount as the conditional
                if (time.milliseconds() <= 300 && time.milliseconds() >= 100) {
                    // attempt to get robot's location based on nav servoPos
                    objectLocator.updateRobotLocation();
                    // add telemetry telling us if robot can see servoPos
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    // strafe angle is only true if the code has been to state STRAFETOTARGET, requiring
                    // the robot to be at about 90 degrees, facing the servoPos
                    // if it is true, update position and change to state FIXANGLE
                    if (angleCorrect) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        auto = AutoState.FIXANGLE;
                        break;
                    }

                    // if not trying to find servoPos, update position
                    if (checkMoveType > 0) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                    }

                    // if adjusting angle, proceed to state FIXANGLE (checkMoveType == 1)
                    // if strafing, proceed to state STRAFETOTARGET (checkMoveType == 2)
                    if (checkMoveType == 1) {
                        auto = AutoState.FIXANGLE;
                        break;
                    } else if (checkMoveType == 2) {
                        auto = AutoState.STRAFETOTARGET;
                        break;
                    }

                    // if servoPos located, update position, reset rotation, and switch to state FIXANGLE
                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        turnCount = 0;
                        auto = AutoState.FIXANGLE;
                    }
                } else if (time.milliseconds() >= 500) {
                    lock = false;

                    // if turnCount == 3, 4, 5, 6 switch to state ROTATECW
                    // if turnCount == 9, switch to state DONE (code is over)
                    // if turnCount == 1, 2, 7, 8 switch to state ROTATECCW
                    // essentially, we want the robot to do two turns cw and the 4 ccw and then
                    // one more cw. turnCount allows us to keep track of where the robot is.
                    if (turnCount >= 3 && turnCount <= 6) {
                        auto = AutoState.ROTATECW;
                    } else if (turnCount == 9) {
                        auto = AutoState.DONE;
                        turnCount = 0;
                        break;
                    } else {
                        auto = AutoState.ROTATECCW;
                    }
                    // increment turnCount
                    turnCount++;
                }

                break;

            // turn robot counterclockwise slowly - for attempting to find nav targets
            case ROTATECCW:
                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counterclockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, 0.2);

                // if time >= 1000 milliseconds, switch state to UPDATE (this both switches the state and stops the robot)
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // turn robot counter clockwise slowly - used if can't find nav targets while turning clockwise
            case ROTATECW:

                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counter clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, -0.2);

                // if time >= 1000 milliseconds, switch state to UPDATE (this both switches the state and stops the robot)
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // get the robot's position based on the nav servoPos and correct the angle until the robot
            // is facing the nav servoPos head on
            case FIXANGLE:
                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if time >= 100 milliseconds, switch state to UPDATE
                // if checkMoveType == 0, set it equal to 1
                if (time.milliseconds() >= 100) {
                    lock = false;
                    if (checkMoveType == 0) {
                        checkMoveType = 1;
                    }
                    auto = AutoState.UPDATE;
                }

                // update position of the robot
                objectLocator.updateRobotLocation();
                lastPos = objectLocator.lastPos;

                // add telemetry for the angle of the robot (in relation to nav servoPos)
                telemetry.addData("angle", lastPos.w);


                // adjust position until angle is within pre-decided threshold
                if (lastPos.w > 91) {
                    drive.driveRobotCentric(0, 0, -0.3);
                } else if (lastPos.w < 89) {
                    drive.driveRobotCentric(0, 0, 0.3);
                    // if angle is within threshold, switch to state STRAFETOTARGET
                } else {
                    // stop driving
                    drive.stop();
                    lock = false;

                    // if angleCorrect == true, switch to state STRAFETOTARGET
                    if (angleCorrect) {
                        auto = AutoState.STRAFETOTARGET;
                    }
                    // if adjusting the angle the first time (angleAdjustCount == 0), switch to state STRAFETOTARGET
                    if (angleAdjustCount == 0) {
                        auto = AutoState.STRAFETOTARGET;
                    } else {
                        // if neither condition is true, set angleAdjustCount = 0 and switch to state SHOOT
                        angleAdjustCount = 0;
                        auto = AutoState.SHOOT;
                    }
                }

                break;

            // finds translational delta (only R/L) and adjusts so that the robot is within a
            // pre-determined threshold
            case STRAFETOTARGET:
                // set angleCorrect = true (so that the states with conditionals requiring it can
                // know that the angle is close enough)
                angleCorrect = true;

                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // adjust position until the robot is within a pre-decided threshold
                if (lastPos.y > desiredY + 3) {
                    drive.driveRobotCentric(0.4, 0, 0);
                } else if (lastPos.y < desiredY - 3) {
                    drive.driveRobotCentric(-0.4, 0, 0);
                // when robot is within threshold, reset variables, stop driving, and switch to state CORRECTX
                } else {
                    angleCorrect = false;
                    drive.stop();
                    lock = false;
                    auto = AutoState.CORRECTX;
                    break;
                }

                // if time >= 500 milliseconds switch to state UPDATE
                if (time.milliseconds() >= 500) {
                    lock = false;

                    // if checkMoveType == 1, set to checkMoveType = 2
                    // the change allows us to not get stuck in an infinite loop and actually
                    // continue the OpMode
                    if (checkMoveType == 1) {
                        checkMoveType = 2;
                    }
                    auto = AutoState.UPDATE;
                }

                // add telemetry for y position
                telemetry.addData("y", lastPos.y);

                break;

            // find distance the robot needs to move behind the shooting line
            case CORRECTX:
                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }
                // update position of the robot
                objectLocator.updateRobotLocation();
                lastPos = objectLocator.lastPos;

                // add telemetry for the angle of the robot (in relation to nav servoPos)
                telemetry.addData("x", lastPos.x);

                // adjust robot position based on pre-determined values
                if (lastPos.x > desiredX + 1) {
                    drive.driveRobotCentric(0, 0.3, 0);
                } else if (lastPos.x < desiredX - 1) {
                    drive.driveRobotCentric(0, -0.3, 0);
                    // if robot is the correct position, switch to state DRIVEBEHINDMID
                } else {
                    drive.stop();
                    auto = AutoState.DRIVEBEHINDMID;
                }

                break;

            // drive to behind shooting line using information from state CORRECTX
            case DRIVEBEHINDMID:
                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive to behind shooting line
                drive.driveRobotCentric(0, 0.3, 0);

                // if time >= 3000 milliseconds, reset variables, stop driving, and switch to state FIXANGLE
                if (time.milliseconds() >= 3000) {
                    lock = false;
                    drive.stop();
                    angleAdjustCount++;
                    auto = AutoState.FIXANGLE;
                }

                break;

            // shoot ring into mid or high goal (depending on how shitty the motor is being)
            case SHOOT:
                // make sure code only runs once, set shooter to on, shooter runs until state is switched, and reset time
                if (!lock) {
                    shooter.set(1);
                    time.reset();
                    lock = true;
                    servoPos = true;
                }

                // if time >= 500 milliseconds, begin toggling shooterServo
                // NOTE: the shooter must be given enough time to get to full power, hence the wait time
                if (time.milliseconds() >= 500) {
                    // set shooterServo = 0, second half of shooting (eg: it closes)
                    shooterServo.setPosition(servoPos ? 0 : 1);
                    time.reset();
                    // toggle servoPos
                    servoPos = !servoPos;
                    // increment servoMoveCount
                    servoMoveCount++;
                } else {
                    // set servo to 1 = first half of shooting (eg: it opens)
                    shooterServo.setPosition(servoPos ? 1 : 0);
                }

                // caps the number of shots at 3 (servoMoveCount keeps track of how many times shooterServo
                // has opened/closed, so the actual rings shot will be half of the value
                // if servoMoveCount == 6, switch to state DRIVETOMID
                if (servoMoveCount == 6) {
                    lock = false;
                    auto = AutoState.DRIVETOMID;
                }

                break;

            // park over shooting line
            case DRIVETOMID:
                // make sure code only runs once and reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if time >= 1800 milliseconds, drive to end up over shooting line
                if (time.milliseconds() >= 1800) {
                    drive.driveRobotCentric(0, -0.3, 0);
                }

                // if time >= 2500 milliseconds, stop robot and switch to state DONE
                if (time.milliseconds() >= 2500) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.DONE;
                }

                break;

            // stops OpMode, either after state DRIVETOMID or in the case of catastrophic failure
            case DONE:
                // stop the entire OpMode, not just motors
                requestOpModeStop();

                break;
        }
    }
}