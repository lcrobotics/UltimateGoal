package org.firstinspires.ftc.teamcode.AutonomousOpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomousOpModes.AutoState;

@Disabled
@Autonomous
public class FarShootAuto extends AutoSuperOpOld {
    // ensure that TURNABIT actually runs
    boolean started = false;
    // declare array to keep track of rotation states
    AutoState[] rotations;
    // start the OpMode in state TURNABIT
    AutoState auto = AutoState.TURNABIT;

    @Override
    public void init() {
        // call init() in AutoSuperOp
        super.init();

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
    }

    public void loop() {
        // make sure that the code runs properly
        // the elapsed time starts in init() and due to a change made, we had to make
        // sure the time was reset before the state machine began
        if (!started) {
            time.reset();
            started = true;
        }

        telemetry.addData("state", auto);

        switch (auto) {
            // turn a small bit to the right (this allows for the robot to be able to see the nav target better)
            case TURNABIT:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // turn away from goals, toward power shots
                drive.driveRobotCentric(0,0, -0.3);

                // stop driving after 300 milliseconds and switch to state DRIVEOVERMID
                if (time.milliseconds() >= 300 && !rotateQuad) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.DRIVEOVERMID;
                } else if(time.milliseconds() >= 100 && rotateQuad) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.SHOOT;
                }

                break;

            // drive from starting position to just past shooting line, allowing camera to see servoPos
            case DRIVEOVERMID:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // begin to drive forward, towards shooting line
                drive.driveRobotCentric(0, -.5, 0);

                // when time >= 3000 milliseconds or the frontLeftDrive's encoder is at about 2849
                // stop driving and switch to state ROTATECCW
                // NOTE: magic number (2849) was found by testing our encoders for which had the most
                // consistent value and picking the one that was lowest. In our case, that was the
                // FrontLeftDrive, and it's lowest value was 2849
                if (time.milliseconds() >= 3000 || 2849 - frontLeftDrive.encoder.getPosition() < 0) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECCW;
                }

                break;

            // stop robot and get location, then use as a springboard to get to other states
            case UPDATE:
                // add telemetry for turnCount value (so we know how many times the robot has turned)
                telemetry.addData("turnCount", turnCount);

                // make sure state only runs once - run at beginning of state, reset time, and stop drive
                if (!lock) {
                    resetDrive();
                    lock = true;
                    time.reset();
                }

                // if time is between 100 milliseconds and 300 milliseconds check for location of robot and proceed into state movement
                // if time >= 500 milliseconds, proceed to turning states by conditionals using turnCount
                if (time.milliseconds() <= 300 && time.milliseconds() >= 100) {
                    // attempt to get robot's location based on nav servoPos
                    objectLocator.updateRobotLocation();
                    // add telemetry telling us if robot can see servoPos
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    // if not trying to find servoPos, update position
                    if (checkMoveType > 0) {
                        lastPos = objectLocator.lastPos;
                    }

                    // if adjusting angle, proceed to state FIXANGLE (checkMoveType == 1)
                    if (checkMoveType == 1) {
                        auto = AutoState.FIXANGLE;
                        lock = false;
                        break;
                    }

                    // if servoPos located, update position, reset rotation, switch to state FIXANGLE
                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        turnCount = 0;
                        auto = AutoState.FIXANGLE;
                    }
                } else if (time.milliseconds() >= 500) {
                    lock = false;
                    // select next rotation based on rotations array (eg: is it in ROTATECCW or ROTATECW)
                    auto = rotations[turnCount];

                    // if turnCount == 9, reset turnCount, else update turnCount
                    if (turnCount == 9) {
                        turnCount = 0;
                    } else {
                        turnCount++;
                    }
                }

                break;

            // turn robot counterclockwise slowly - for attempting to find nav targets
            case ROTATECCW:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counterclockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, 0.2);

                // if time >= 1000 milliseconds, switch to state UPDATE
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // turn robot clockwise slowly - used if can't find nav targets while turning counterclockwise
            case ROTATECW:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, -0.2);

                // if time >= 1000 milliseconds, switch to state UPDATE
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // get the robot's position based on the nav servoPos and correct the angle until the robot
            // is facing the nav servoPos head on
            case FIXANGLE:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // the angle that we want to end up with
                double desiredAngle = 79;
                // adjust desiredAngle, so that the first time it runs, it goes further towards the center
                if (angleAdjustCount == 0) {
                    desiredAngle = 140;
                }

                // time that we want to rotate before checking position again using nav targets
                double rotatingTime = 100;
                // if we have a recorded last position
                if (lastPos != null) {
                    // factor used to calculate approximate rotation time - NOT TESTED YET
                    double factor = 30;
                    rotatingTime = Math.max(100, Math.abs(desiredAngle - lastPos.w) * factor);
                }

                // if time > rotatingTime, end state and switch state to UPDATE
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
                    // stop driving, reset encoders, end state
                    resetDrive();
                    lock = false;

                    // if adjusting the angle the first time (angleAdjustCount == 0), switch to state CENTER
                    if (angleAdjustCount == 0) {
                        auto = AutoState.CENTER;
                    } else if (angleAdjustCount == 1) { // if adjusting for the second time, switch to state DRIVEBEHINDMID
                        auto = AutoState.DRIVEBEHINDMID;
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

                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive forwards
                drive.driveRobotCentric(0, -.5, 0);

                // if time >= 1200 milliseconds, stop drive, and switch to state ROTATECW
                if(time.milliseconds() >= 1200) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECW;
                }

                break;

            // drive to behind shooting line
            case DRIVEBEHINDMID:
                // make sure state only runs once - run at beginning of state, reset time, and run
                // intake backwards - to make sure we don't run into the ring
                if (!lock) {
                    time.reset();
                    intake.set(-1);
                    lock = true;
                }

                // drive to behind shooting line
                drive.driveRobotCentric(0, 0.42, 0);

                // if time >= 3100 milliseconds, stop intake, stop drive, and switch to state SHOOT
                if (time.milliseconds() >= 3100) {
                    intake.set(0);
                    lock = false;
                    resetDrive();
                    rotateQuad = true;
                    auto = AutoState.TURNABIT;
                }

                break;

            // shoot ring into mid goal
            case SHOOT:
                // make sure state only runs once - run at beginning of state, reset time, turn on
                // shooter, initialize boolean servoPos
                if (!lock) {
                    shooter.set(1);
                    servoPos = true;
                    time.reset();
                    lock = true;
                }

                // if time >= 1750 milliseconds, begin toggling shooterServo
                // NOTE: the shooter must be given enough time to get to full power, hence the wait time
                if (time.milliseconds() >= 1750) {
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
                    shooter.set(0);
                    shooterServo.setPosition(0);
                    auto = AutoState.DRIVETOMID;
                }

                break;

            // park over shooting line
            case DRIVETOMID:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if first time in DRIVETOMID, drive farther over line (to drop wobble - park == 0)
                // if second time in DRIVETOMID, drive backwards to properly park (park == 1)
                if (park == 0) {
                    // drive farther over shooting line
                    drive.driveRobotCentric(0, -0.43, 0);

                    // if time >= 1900 milliseconds, drive to end up over shooting line
                    // reset encoders and stop drive motors, increment park, switch state to
                    // DROPWOBBLE
                    if (time.milliseconds() >= 1900) {
                        lock = false;
                        resetDrive();
                        park++;
                        auto = AutoState.DROPWOBBLE;
                    }
                } else if (park == 1) {
                    // drive to a bit more on the line
                    drive.driveRobotCentric(0, 0.3, 0);

                    // if time >= 700 milliseconds, stop drive motors & reset encoders, switch state
                    // to DONE
                    if(time.milliseconds() >= 700) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                break;

            // drive forward and drop the wobble goal in box B
            case DROPWOBBLE:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    lock = true;
                    time.reset();
                    break;
                }

                // drive forward
                drive.driveRobotCentric(0, 0, 0.3);
                // if time >= 600 milliseconds, stop driving, release wobble goal, and switch to state
                // DRIVETOMID
                if (time.milliseconds() >= 600) {
                    lock = false;
                    resetDrive();
                    topHook.setPosition(0);
                    auto = AutoState.DRIVETOMID;
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