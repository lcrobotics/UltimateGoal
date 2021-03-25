package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoClose extends AutoSuperOp {
    // ensure that TURNABIT actually runs
    boolean started = false;
    // declare array to keep track of rotation states
    AutoState[] rotations;
    // start the OpMode in state TURNABIT
    AutoState auto = AutoState.TURNABIT;

    @Override
    public void init() {
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

        // add telemetry for state the code is in
        telemetry.addData("state", auto);
        telemetry.addData("angleAdjustCount", angleAdjustCount);

        switch (auto) {
            // turn a small but to the right (this allows for the robot to be able to see the nav target better)
            case TURNABIT:
                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state DRIVEOVERMID
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // turn
                drive.driveRobotCentric(0,0, -0.3);

                // set lock to true if time >= 300 milliseconds, prompting above if statement
                if (time.milliseconds() >= 300) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.DRIVEOVERMID;
                }

                break;

            // drive from starting position to just past shooting line, allowing camera to see servoPos
            case DRIVEOVERMID:
                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state ROTATECCW
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // begin to drive forward, towards shooting line
                drive.driveRobotCentric(0, -.5, 0);

                // when time >= 3000 milliseconds or the frontLeftDrive's encoder is at about 2849
                // set lock to true and prompt above if statement
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

                // ensure code runs once, reset encoders and stop motors and reset time
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    resetDrive();
                    lock = true;
                    time.reset();
                }

                // if time is between 100 milliseconds and 300 milliseconds check for location of robot and proceed into state movement
                // if time > 500 milliseconds, proceed to turning states by conditionals using turnCount
                if (time.milliseconds() <= 300 && time.milliseconds() >= 100) {
                    // attempt to get robot's location based on nav servoPos
                    objectLocator.updateRobotLocation();
                    // add telemetry telling us if robot can see servoPos
                    telemetry.addData("VISIBLE", objectLocator.targetVisible);

                    // if robot is correcting in the horizontal axis, go to FIXANGLE to adjust angle
                    if (correctingHorizontal) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        auto = AutoState.FIXANGLE;
                        break;
                    }

                    // if not trying to find servoPos, update position and set lock to false
                    if (checkMoveType > 0) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                    }

                    // if adjusting angle, proceed to state FIXANGLE (checkMoveType == 1)
                    // if correcting horizontal, proceed to state CORRECTHORIZONTAL (checkMoveType == 2)
                    if (checkMoveType == 1) {
                        auto = AutoState.FIXANGLE;
                        break;
                    } else if (checkMoveType == 2) {
                        auto = AutoState.CORRECTHORIZONTAL;
                        break;
                    }

                    // if servoPos located, update position, reset rotation, switch to state FIXANGLE
                    // and set lock to true (prompting the first if statement)
                    if (objectLocator.targetVisible) {
                        lastPos = objectLocator.lastPos;
                        lock = false;
                        turnCount = 0;
                        auto = AutoState.FIXANGLE;
                    }
                } else if (time.milliseconds() > 500) {
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
                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state UPDATE
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn counterclockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, 0.2);

                // if time > 1000 milliseconds, set lock to true, prompting above if statement
                if (time.milliseconds() > 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // turn robot clockwise slowly - used if can't find nav targets while turning counterclockwise
            case ROTATECW:
                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state UPDATE
                // NOTE: this code only runs at the very end of the state, when lock is set to tru
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // turn clockwise (according to robot, not field view)
                drive.driveRobotCentric(0, 0, -0.2);

                // if time >= 1000 milliseconds, set lock to true, prompting above if statement
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.UPDATE;
                }

                break;

            // get the robot's position based on the nav servoPos and correct the angle until the robot
            // is facing the nav servoPos head on
            case FIXANGLE:
                // ensure code runs once, reset time
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // the angle that we want to end up with
                double desiredAngle = 90;
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

                    // go back to CORRECTHORIZONTAL if robot is currently correcting in the horizontal axis
                    if (correctingHorizontal) {
                        auto = AutoState.CORRECTHORIZONTAL;
                        break;
                    }

                    // if adjusting the angle the first time (angleAdjustCount == 0), switch to state CENTER
                    if (angleAdjustCount == 0) {
                        auto = AutoState.CENTER;
                    } else if (angleAdjustCount == 1) { // if adjusting for the second time, switch to state CORRECTHORIZONTAL
                        auto = AutoState.CORRECTHORIZONTAL;
                    } else if (angleAdjustCount == 2) { // if adjusting for the third time, switch to state DRIVEBEHINDMID
                        auto = AutoState.DRIVEBEHINDMID;
                    } else if (angleAdjustCount == 3) { // if adjusting for the fourth time, switch to state SHOOT & turn on shooter
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

                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state ROTATECW
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive forwards
                drive.driveRobotCentric(0, -.5, 0);

                // if time >= 1200 milliseconds, set lock to true and prompt above if statement
                if(time.milliseconds() >= 1200) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECW;
                }

                break;

            // finds translational delta (only R/L) and adjusts so that the robot is within a
            // pre-determined threshold
            case CORRECTHORIZONTAL:
                // set correctingHorizontal = true (so that the states with conditionals requiring it can
                // know that the robot is correcting in its horizontal axis)
                correctingHorizontal = true;

                // make sure code only runs once
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // adjust position until the robot is within a pre-decided threshold
                if (lastPos.y > desiredY + 2) {
                    drive.driveRobotCentric(0.4, 0, 0);
                } else if (lastPos.y < desiredY - 2) {
                    drive.driveRobotCentric(-0.4, 0, 0);
                    // when robot is within threshold, reset variables, stop driving, and switch to state FIXANGLE
                } else {
                    correctingHorizontal = false;
                    drive.stop();
                    lock = false;
                    auto = AutoState.FIXANGLE;
                    break;
                }

                // if time > 500 milliseconds switch to state UPDATE
                if (time.milliseconds() > 500) {
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

            // drive to behind shooting line
            case DRIVEBEHINDMID:
                // ensure code runs once, reset encoders and stop motors, reset time and switch to
                // state FIXANGLE
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // drive to behind shooting line
                drive.driveRobotCentric(0, 0.3, 0);

                // if time >= 3100 milliseconds, set lock to true and prompt above if statement
                if (time.milliseconds() >= 3300) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.FIXANGLE;
                }

                break;

            // shoot ring into mid or high goal (depending on how unreliable the motor is being)
            case SHOOT:
                // ensure code runs once, reset encoders and stop motors, reset time, turn off
                // shooter & shooter servo and switch to
                // state DRIVETOMID
                // NOTE: this code only runs at the very end of the state, when lock is set to tru
                if (!lock) {
                    servoPos = true;
                    shooter.set(1);
                    time.reset();
                    lock = true;
                }

                // if time >= 1500 milliseconds, begin toggling shooterServo
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
                // ensure code runs once, reset time
                // NOTE: this code only runs at the very end of the state, when lock is set to tru
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if first time in DRIVETOMID, drive farther over line (to drop wobble)
                // if second time in DRIVETOMID, drive backwards to properly park
                if (park == 0) {
                    // drive farther over shooting line
                    drive.driveRobotCentric(0, -0.3, 0);

                    // if time >= 1400 milliseconds, drive to end up over shooting line
                    // reset encoders and stop drive motors, increment park, switch state to
                    // DROPWOBBLE, and set lock to true (prompting first if statement)
                    if (time.milliseconds() >= 1600) {
                        lock = false;
                        resetDrive();
                        park++;
                        auto = AutoState.DROPWOBBLE;
                    }
                } else if (park == 1) {
                    // drive to a bit more on the line
                    drive.driveRobotCentric(0, 0.3, 0);

                    // if time >= 800 milliseconds, stop drive motors & reset encoders, switch state
                    // to DONE, and set lock to true (prompting first if statement)
                    if(time.milliseconds() >= 900) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                break;

            // drive forward and drop the wobble goal in box B
            case DROPWOBBLE:
                // ensure code runs once, reset encoders and stop motors, reset time, trigger topHook
                // (drop wobble goal) and switch to
                // state DRIVETOMID
                // NOTE: this code only runs at the very end of the state, when lock is set to true
                if (!lock) {
                    lock = true;
                    time.reset();
                    break;
                }

                // drive forward
                drive.driveRobotCentric(0, 0, 0.3);
                // if time >= 600 milliseconds, set lock to true and prompt above if statement
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

    // stop all drive motors, set RunMode to STOP_AND_RESET_ENCODER, then set to RUN_WITHOUT_ENCODER
    void resetDrive() {
        drive.stop();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // set drive motors to user specified RunMode (used in resetDrive() for clarity)
    void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.motor.setMode(mode);
        frontRightDrive.motor.setMode(mode);
        backRightDrive.motor.setMode(mode);
        backLeftDrive.motor.setMode(mode);
    }
}
