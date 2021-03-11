package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RingAuto extends AutoSuperOp {
    // declare array to keep track of rotation states
    AutoState[] rotations;
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
                // sequence of rotations that will be used after DRIVEOVERMID is completed
                rotations = new AutoState[] {
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.DONE,
                };

                // make sure this only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // begin to drive forward, towards shooting line
                drive.driveRobotCentric(0, -0.5, 0);

                // when time > 3000 milliseconds, reset lock, stop robot, and go to state ROTATECCW
                if (time.milliseconds() >= 3000) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.ROTATECCW;
                }

                break;

            // stop robot and get location, then use as a springboard to get to other states
            case UPDATE:
                // add telemetry for turnCount value (so we know how many times the robot has turned)
                telemetry.addData("turnCount", turnCount);

                // make sure code only runs worse
                if (!lock) {
                    drive.stop();
                    time.reset();
                    lock = true;
                }

                // if time is between 100 milliseconds and 300 milliseconds check for location of robot and proceed into state movement
                // if time > 500 milliseconds, proceed to turning states by conditionals using turnCount
                if (time.milliseconds() <= 300 && time.milliseconds() >= 100) {
                    // attempt to get robot's location based on nav servoPos
                    objectLocator.updateRobotLocation();
                    // add telemetry telling us if robot can see servoPos
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    // if not trying to find servoPos, update position
                    if (checkMoveType > 0) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                    }

                    // if adjusting angle, proceed to state FIXANGLE (checkMoveType == 1)
                    if (checkMoveType == 1) {
                        auto = AutoState.FIXANGLE;
                        break;
                    }

                    // if servoPos located, update position, reset rotation, and switch to state FIXANGLE
                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        turnCount = 0;
                        auto = AutoState.FIXANGLE;
                    }
                } else if (time.milliseconds() > 500) {
                    lock = false;
                    // select next rotation based on rotations array
                    auto = rotations[turnCount];

                    // update turnCount
                    if (turnCount == 9) {
                        turnCount = 0;
                    } else {
                        turnCount++;
                    }
                }

                break;

            // turn robot clockwise slowly - for attempting to find nav targets
            case ROTATECCW:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, 0.2);

                // if time > 1000 milliseconds, switch state to UPDATE (this both switches the state and stops the robot)
                if (time.milliseconds() > 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // turn robot counter clockwise slowly - used if can't find nav targets while turning clockwise
            case ROTATECW:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counter clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, -0.2);

                // if time > 1000 milliseconds, switch state to UPDATE (this both switches the state and stops the robot)
                if (time.milliseconds() > 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // get the robot's position based on the nav servoPos and correct the angle until the robot
            // is facing the nav servoPos head on
            case FIXANGLE:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // the angle that we want to end up with
                double desiredAngle = 90;
                // adjust desiredAngle, so that the first time it runs, it goes further towards the center
                if (angleAdjustCount == 0) {
                    desiredAngle = 120;
                }

                // time that we want to rotate before checking position again using nav targets
                double rotatingTime = 100;

                // if we have a recorded last position
                if (lastPos != null) {
                    // factor used to calculate approximate rotation time - NOT TESTED YET
                    double factor = 30;
                    rotatingTime = Math.max(100, Math.abs(desiredAngle - lastPos.w) * factor);
                }

                // if time > rotatingTime, switch state to UPDATE
                if (time.milliseconds() > rotatingTime) {
                    lock = false;
                    // if checkMoveType == 0, set it equal to 1
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
                if (lastPos.w > desiredAngle + 1) {
                    drive.driveRobotCentric(0, 0, -0.3);
                } else if (lastPos.w < desiredAngle - 1) {
                    drive.driveRobotCentric(0, 0, 0.3);
                } else {
                    // stop driving
                    drive.stop();
                    lock = false;

                    // if adjusting the angle the first time (angleAdjustCount == 0), switch to state CENTER
                    if (angleAdjustCount == 0) {
                        auto = AutoState.CENTER;
                    } else if (angleAdjustCount == 1) { // if adjusting for the second time, switch to state DRIVEBEHINDMID
                        auto = AutoState.DRIVEBEHINDMID;
                    } else if (angleAdjustCount == 2) { // if adjusting for the third time, switch to state SHOOT
                        auto = AutoState.SHOOT;
                    }
                    // update angleAdjustCount
                    angleAdjustCount++;
                }

                break;

            // drive forward and stop in box B (robot will be going diagonal from driver perspective)
            case CENTER:
                // sequence of rotations that will be used after CENTER is completed
                rotations = new AutoState[]{
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECCW,
                        AutoState.ROTATECW,
                        AutoState.ROTATECW,
                        AutoState.DONE,
                };

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive forwards
                drive.driveRobotCentric(0, -.5, 0);

                // if time > 1000 milliseconds, stop and switch to state ROTATECW
                if(time.milliseconds() > 1000) {
                    drive.stop();
                    auto = AutoState.ROTATECW;
                }

                break;

            // drive to behind shooting line
            case DRIVEBEHINDMID:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive to behind shooting line
                drive.driveRobotCentric(0, 0.3, 0);

                // if time > 3000 milliseconds, reset variables, stop driving, and switch to state FIXANGLE
                if (time.milliseconds() >= 3000) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.FIXANGLE;
                }

                break;

            // shoot ring into mid or high goal (depending on how shitty the motor is being)
            case SHOOT:
                // make sure code only runs once, set shooter to on, shooter runs until state is switched
                if (!lock) {
                    shooter.set(1);
                    time.reset();
                    lock = true;
                    servoPos = true;
                }

                // if time > 500 milliseconds, begin toggling shooterServo
                // NOTE: the shooter must be given enough time to get to full power, hence the wait time
                if (time.milliseconds() > 500) {
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
                    auto = AutoState.DROPWOBBLE;
                }

                break;

            // drive forward and drop the wobble goal in box B
            case DROPWOBBLE:
                // drive forward
                drive.driveRobotCentric(0, 0.5, 0);
                // if time > 2500 milliseconds, stop, release goal, and switch to state DRIVETOMID
                if (time.milliseconds() > 2500) {
                    drive.stop();
                    topHook.setPosition(1);
                    auto = AutoState.DRIVETOMID;
                }

                // park over shooting line
            case DRIVETOMID:
                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if time > 1800 milliseconds, drive to end up over shooting line
                if (time.milliseconds() >= 1800) {
                    drive.driveRobotCentric(0, 0.3, 0);
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