package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class CleanAuto extends AutoSuperOp {
    // initialize boolean that ensures that loop() only runs once
    // and that the program starts in the right state
    boolean started = false;
    // start OpMode in state DETECT
    AutoState auto = AutoState.DETECT;

    public void init() {
        // run AutoSuperOp's init()
        super.init();
    }

    @Override
    public void loop() {
        // ensure that loop() only runs once and reset time (elapsed time is still counted in init())
        if (!started) {
            time.reset();
            started = true;
        }

        // add telemetry (these stay throughout the entire OpMode)
        telemetry.addData("state", auto);
        telemetry.addData("time", time);
        telemetry.addData("shoot", shoot);
        telemetry.addData("voltage", getBatteryVoltage());
        telemetry.addData("rings", numRings);

        // declare state machine auto and it's states
        switch(auto) {
            // runs once at the beginning of the OpMode to detect the number of rings on the field
            case DETECT:
                // ensure that DETECT runs once per time in the state (runs at the end of the state,
                // resets time for the next state)
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // call detect from AutoSuperOp
                detect();

                // wait 2000 milliseconds so the code has a chance to see the stack of rings, then
                // set lock to false and switch to state DRIVEABIT
                if (time.milliseconds() >= 2000) {
                    lock = false;
                    auto = AutoState.DRIVEABIT;
                }

                break;

            // runs after state DETECT and if the voltage is greater than 12.5 while there is a four
            // stack of rings. This state drives forward a small amount
            case DRIVEABIT:
                // ensure that DRIVEABIT runs once per time in the state (runs at the end of the state,
                // resets time for the next state)
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // if it's the first time in the state, drive forward for 200 milliseconds then
                // stop motors, reset encoders, set lock to false, and switch to state SHOOT
                // if rotateQuad is true (only happens when there is a four stack and after state
                // DRIVEOVERMID), drive forward for 700 milliseconds then stop motors, reset encoders,
                // set lock to false, and switch to state DROPWOBBLE
                if (!rotateQuad) {
                    // drive forward at -.3 power
                    drive.driveRobotCentric(0,-.3,0);
                    // wait 200 milliseconds then stop motors, reset encoders, set lock to false,
                    // and switch to state SHOOT
                    if (time.milliseconds() >= 200) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.SHOOT;
                    }
                } else {
                    // drive forward at -.35
                    drive.driveRobotCentric(0,-.35,0);
                    // wait 700 milliseconds then stop motors, reset encoders, set lock to false,
                    // and switch to state DROPWOBBLE
                    if (time.milliseconds() >= 700) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }


                break;

            // runs once, right after DRIVEABIT, to shoot the preloaded rings
            case SHOOT:
                // ensure that SHOOT runs once per time in the state (runs at the end of the state,
                // resets time for the next state, sets the shooter to run on velocity, and sets
                // boolean servoPos to true)
                if (!lock) {
                    servoPos = true;
                    time.reset();
                    lock = true;
                    // set the shooter to run on it's max ticks per second (using velocity - requires
                    // encoder) by casting to DcMotorEx. we set the motor to velocity instead of using
                    // pure battery power because the encoder ticks are less reliant on the battery's
                    // voltage, allowing for a slightly more consistent shooter.
                    ((DcMotorEx)shooter.motor).setVelocity(shooter.motor.getMotorType().getAchieveableMaxTicksPerSecond());
                }

                // wait 4000 milliseconds to give the shooter time to spin up to full power, then
                // reset time so the code is clearer, and set shoot to true to ensure that this
                // section state SHOOT only runs once
                if (time.milliseconds() >= 4000 && !shoot) {
                    time.reset();
                    shoot = true;
                }

                // wait 1500 milliseconds for shooter to recover, then begin toggling the shooterServo
                // this only runs after shoot is true, so that the program doesn't skip the original
                // waiting period (runs after each ring is shot and resets the time after each recovery
                // period) if shoot is true but the code has not waited 1500 milliseconds, shoot the
                // ring, and then wait
                if (time.milliseconds() >= 1500 && shoot) {
                    // toggle servo (bring back after shooting)
                    shooterServo.setPosition(servoPos ? 0 : 1);
                    // reset the time
                    time.reset();
                    // reset the boolean used to toggle the servo
                    servoPos = !servoPos;
                    // iterate servoMoveCount so that the state knows when to end
                    servoMoveCount++;
                } else if (shoot) {
                    // toggle servo (bring out to shoot ring)
                    shooterServo.setPosition(servoPos ? 1 : 0);
                }

                // once the servo has moved six times (all three rings are shot), set lock to false,
                // turn off shooter, set shooterServo to 0 (bring back to it's rest point), and switch
                // to state ROTATECW
                if (servoMoveCount == 6) {
                    lock = false;
                    shooter.set(0);
                    shooterServo.setPosition(0);
                    auto = AutoState.ROTATECW;
                }

                break;

            // runs at the beginning of the OpMode, after state SHOOT and runs at some point depending
            // on the number of rings on the field. turns clockwise based on the robot, in
            // this OpMode clockwise is to the left
            case ROTATECW:
                // ensure that ROTATECW runs once per time in the state (runs at the end of the state,
                // resets time for the next state)
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // booleans all being ensures that it is the first time in the state. if it is the
                // first time in the state, check battery voltage and turn different times based on
                // the voltage
                if (!rotateZeroCW && !rotateSingle && !rotateQuad) {
                    // check if the battery's voltage is greater than 12.5
                    if (getBatteryVoltage() >= 12.5) {
                        drive.driveRobotCentric(0,0,-.34);
                        if (time.milliseconds() >= 300) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVEOVERMID;
                        }
                    } else {
                        drive.driveRobotCentric(0,0,-.34);
                        if (time.milliseconds() >= 500) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVEOVERMID;
                        }
                    }
                }

                if(rotateZeroCW) {
                    drive.driveRobotCentric(0,0,-.32);
                    if (time.milliseconds() >= 675) {
                        resetDrive();
                        lock = false;
                        rotateZeroCCW = true;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                if (rotateSingle) {
                    if (time.milliseconds() >= 300) {
                        drive.driveRobotCentric(0, 0, -.32);
                        if (time.milliseconds() >= 900) {
                            resetDrive();
                            if (time.milliseconds() >= 1200) {
                                lock = false;
                                auto = AutoState.DRIVETOMID;
                            }
                        }
                    }
                }

                if(rotateQuad) {
                    if (time.milliseconds() >= 300) {
                        drive.driveRobotCentric(0,0,-.4);
                        if (getBatteryVoltage() >= 12.5) {
                            if (time.milliseconds() >= 1100) {
                                resetDrive();
                                lock = false;
                                auto = AutoState.DRIVEABIT;
                            }
                        } else {
                            if (time.milliseconds() >= 1300) {
                                resetDrive();
                                lock = false;
                                auto = AutoState.DROPWOBBLE;
                            }
                        }
                    }
                }

                break;

            case DRIVEOVERMID:
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                if (numRings == 0) {
                    drive.driveRobotCentric(0,-.42,0);
                    if(time.milliseconds() >= 2600) {
                        lock = false;
                        resetDrive();
                        rotateZeroCW = true;
                        auto = AutoState.ROTATECW;
                    }
                }

                if (numRings == 1) {
                    drive.driveRobotCentric(0, -0.48, 0);
                    if (getBatteryVoltage() >= 12.6) {
                        if (time.milliseconds() >= 3200) {
                            lock = false;
                            rotateSingle = true;
                            resetDrive();
                            auto = AutoState.DROPWOBBLE;
                        }
                    } else {
                        if (time.milliseconds() >= 3400) {
                            lock = false;
                            rotateSingle = true;
                            resetDrive();
                            auto = AutoState.ROTATECCW;
                        }
                    }
                }

                if (numRings == 4) {
                    if (getBatteryVoltage() >= 12.5) {
                        drive.driveRobotCentric(0, -.53, 0);
                        if (time.milliseconds() >= 3600) {
                            lock = false;
                            resetDrive();
                            rotateQuad = true;
                            auto = AutoState.ROTATECW;
                        }
                    } else {
                        drive.driveRobotCentric(0, -.53, 0);
                        if (time.milliseconds() >= 3800) {
                            lock = false;
                            resetDrive();
                            rotateQuad = true;
                            auto = AutoState.ROTATECW;
                        }
                    }
                }

                break;

            case ROTATECCW:
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                if (rotateZeroCCW) {
                    if (time.milliseconds() >= 200) {
                        drive.driveRobotCentric(0, 0, .36);
                        if (time.milliseconds() >= 1300) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVETOMID;
                        }
                    }
                }

                if (rotateSingle) {
                    drive.driveRobotCentric(0, 0, 0.45);
                    if (time.milliseconds() >= 300) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                if (rotateQuad) {
                    drive.driveRobotCentric(0,0, .36);
                    if(time.milliseconds() >= 700) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVETOMID;
                    }
                }

                break;

            case DROPWOBBLE:
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                if(time.milliseconds() >= 1000 && (rotateZeroCCW || rotateQuad)) {
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECCW;
                }

                if (time.milliseconds() >= 1000 && rotateSingle) {
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECW;
                }

                break;

            case DRIVETOMID:
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                if (numRings == 0) {
                    drive.driveRobotCentric(0, -0.4, 0);
                    if(time.milliseconds() >= 1050) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                if (numRings == 1) {
                    drive.driveRobotCentric(0, 0.35, 0);
                    if (time.milliseconds() >= 950) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                if (numRings == 4) {
                    if (getBatteryVoltage() >= 12.5) {
                        drive.driveRobotCentric(0, .45, 0);
                        if (time.milliseconds() >= 1000) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    } else {
                        drive.driveRobotCentric(0, .6, 0);
                        if (time.milliseconds() >= 1400) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    }
                }

                break;

            case DONE:
                requestOpModeStop();

                break;
        }
    }
}