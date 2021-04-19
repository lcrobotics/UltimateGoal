package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class RingDetectStartVision extends AutoSuperOp {
    // ensure that DETECT actually runs
    boolean started = false;
    // start the OpMode in state DETECT
    AutoState auto = AutoState.DETECT;


    public void init() {
        // run AutoSuperOp's init()
        super.init();
    }

    @Override
    public void loop() {
        // make sure that the code runs properly
        // the elapsed time starts in init() and due to a change made, we had to make
        // sure the time was reset before the state machine began
        if (!started) {
            time.reset();
            started = true;
        }

        // add telemetry
        telemetry.addData("state", auto);
        telemetry.addData("time", time);
        telemetry.addData("shoot", shoot);
        telemetry.addData("voltage", getBatteryVoltage());
        telemetry.addData("rings", numRings);

        switch(auto) {
            case DETECT:
                // make sure code only runs once and reset the time
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // run detect method from AutoSuperOp
                detect();

                // give vision code 2000 milliseconds to run, then switch to state DRIVEABIT and
                // reset lock
                if (time.milliseconds() >= 2000) {
                    lock = false;
                    auto = AutoState.DRIVEABIT;
                }

                break;

            // drive forward a small bit at the very beginning of OpMode
            case DRIVEABIT:
                // make sure code only runs once and reset the time
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // drive forward
                drive.driveRobotCentric(0,-.3,0);
                // if time >= 150 milliseconds stop driving, set lock to false, and switch to state SHOOT
                if (time.milliseconds() >= 300) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.SHOOT;
                }

                break;

            // shoot rings - this shoots at the mid goal from the starting position
            case SHOOT:
                // make sure state only runs once - run at beginning of state, reset time, turn on
                // shooter, initialize boolean servoPos
                // NOTE: the shooter is running at the highest possible velocity, made possible by an
                // encoder
                if (!lock) {
                    servoPos = true;
                    time.reset();
                    lock = true;
                    // cast shooter to DcMotorEx to set velocity to the max encoder tick per second
                    // this runs faster than simply setting the power to 1, as the direct set relies
                    // on battery voltage. velocity does not rely on this as heavily
                    ((DcMotorEx)shooter.motor).setVelocity(shooter.motor.getMotorType().getAchieveableMaxTicksPerSecond());
                }

                // give shooter time to spin up to full power
                // use shoot to make sure this doesn't run again and reset time so shooter code works
                if (time.milliseconds() >= 4000 && !shoot) {
                    time.reset();
                    shoot = true;
                }

                // if time >= 1500 milliseconds and shoot is true (makes sure it doesn't shoot
                // before the time is up), begin toggling shooterServo
                // NOTE: the shooter must be given enough time to get to full power, hence the wait time
                if (time.milliseconds() >= 1500 && shoot) {
                    telemetry.addData("finished shoot section", null);
                    // set shooterServo = 0, second half of shooting (eg: it closes)
                    shooterServo.setPosition(servoPos ? 0 : 1);
                    // reset time so this can run again
                    time.reset();
                    // toggle servoPos
                    servoPos = !servoPos;
                    // increment servoMoveCount
                    servoMoveCount++;
                } else if (shoot) {
                    // set servo to 1 = first half of shooting (eg: it opens)
                    shooterServo.setPosition(servoPos ? 1 : 0);
                }

                // caps the number of shots at 3 (servoMoveCount keeps track of how many times shooterServo
                // has opened/closed, so the actual rings shot will be half of the value
                // if servoMoveCount == 6, switch to state ROTATECW
                if (servoMoveCount == 6) {
                    lock = false;
                    shooter.set(0);
                    shooterServo.setPosition(0);
                    auto = AutoState.ROTATECW;
                }

                break;

            // turn to the left
            case ROTATECW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // when there are zero rings on the field, we need to rotate around goal to park, the
                // boolean makes sure we only run the turns we want at each time (zeroOne is only set
                // to true if the number of rings is 0 in state DRIVETOMID)
                // if zeroOne is false, (should be until last time in state) run the following code
                if (!zeroOne && !single) {
                   // if turn is false (declared as false in AutoSuperOp) run this code - should be run
                   // on first time in state
                   // if the battery is greater than 13 volts, it needs to turn less, due to battery power
                   // messing with the motors (that's why we have these if statements)
                   if (getBatteryVoltage() >= 13 && !turn) {
                       // turn to the left
                       drive.driveRobotCentric(0,0,.34);
                       // when time >= 300 milliseconds, reset drive, set lock to false, and switch to state
                       // DRIVETOMID
                       if (time.milliseconds() >= 300) {
                           resetDrive();
                           lock = false;
                           auto = AutoState.DRIVETOMID;
                       }
                   } else if (!turn) {
                       // turn to the left
                       drive.driveRobotCentric(0,0,.34);
                       // when time >= 350 milliseconds, reset drive, set lock to false, and switch to state
                       // DRIVETOMID
                       if (time.milliseconds() >= 400) {
                           resetDrive();
                           lock = false;
                           auto = AutoState.DRIVETOMID;
                       }
                   }

                   // if turn is true (set to true in state DRIVETOMID when there are 4 rings) - should
                   // run on second time in state
                   if(turn) {
                       // turn left
                       drive.driveRobotCentric(0,0,.36);
                       // when time >= 600 milliseconds, reset drive, set lock to false, and
                       // switch to state DROPWOBBLE
                       if (time.milliseconds() >= 600) {
                           resetDrive();
                           lock = false;
                           auto = AutoState.DROPWOBBLE;
                       }
                   }
                }

                if(zeroOne) {
                    // turn left
                    drive.driveRobotCentric(0,0,.32);
                    // when time >= 550 milliseconds, reset drive, set lock to false, set zeroTwo
                    // to true (used when there are zero rings on the field in ROTATECCW), and switch
                    // to state DROPWOBBLE
                    if (time.milliseconds() >= 575) {
                        resetDrive();
                        lock = false;
                        zeroTwo = true;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                if (single) {
                    if (time.milliseconds() >= 200) {
                        drive.driveRobotCentric(0, 0, .32);
                        if (time.milliseconds() >= 800) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVETOMID;
                        }
                    }
                }
                break;

            // park over shooting line
            case DRIVETOMID:
                // make sure state only runs once - run at beginning of state, reset time
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // run this code if there are 0 rings on the field and wobble goal is dropped in
                // box A
                if (numRings == 0) {
                    // if first time in DRIVETOMID, drive farther over line (to drop wobble - park == 0)
                    // if second time in DRIVETOMID, drive backwards to properly park (park == 1)
                    if (park == 0) {
                        // drive farther over shooting line
                        drive.driveRobotCentric(0,-.4,0);

                        // if time >= 2800 milliseconds, drive to end up over shooting line
                        // reset encoders and stop drive motors, increment park, switch state to
                        // ROTATECW, and set zeroOne to true
                        if(time.milliseconds() >= 2400) {
                            lock = false;
                            resetDrive();
                            park++;
                            zeroOne = true;
                            auto = AutoState.ROTATECW;
                        }
                    } else if (park == 1) {
                        // drive to a bit more on the lineg
                        drive.driveRobotCentric(0, -0.4, 0);

                        // if time >= 600 milliseconds, stop drive motors & reset encoders, switch state
                        // to DONE
                        if(time.milliseconds() >= 850) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    }
                }

                // run this code if there is 1 ring on the field and wobble goal is dropped in
                // box B
                if (numRings == 1) {
                    // if first time in DRIVETOMID, drive farther over line (to drop wobble - park == 0)
                    // if second time in DRIVETOMID, drive backwards to properly park (park == 1)
                    if (park == 0) {
                        // drive farther over shooting line
                        drive.driveRobotCentric(0, -0.48, 0);

                        // if time >= 3000 milliseconds, drive to end up over shooting line
                        // reset encoders and stop drive motors, increment park, switch state to
                        // ROTATECCW
                        if (time.milliseconds() >= 3200) {
                            lock = false;
                            single = true;
                            resetDrive();
                            park++;
                            auto = AutoState.ROTATECCW;
                        }
                    } else if (park == 1) {
                        // drive to a bit more on the line
                        drive.driveRobotCentric(0, 0.35, 0);

                        // if time >= 750 milliseconds, stop drive motors & reset encoders, switch state
                        // to DONE
                        if (time.milliseconds() >= 950) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    }
                }

                // run this code if there are 4 rings on the field and wobble goal is dropped in
                // box C
                if (numRings == 4) {
                    // if first time in DRIVETOMID, drive farther over line (to drop wobble - park == 0)
                    // if second time in DRIVETOMID, drive backwards to properly park (park == 1)
                    if (park == 0) {
                        // drive farther over shooting line
                        drive.driveRobotCentric(0, -.48, 0);

                        // if time >= 3800 milliseconds, drive to end up over shooting line
                        // reset encoders and stop drive motors, increment park, switch state to
                        // ROTATECW, and set turn to true
                        if (time.milliseconds() >= 3800) {
                            lock = false;
                            resetDrive();
                            park++;
                            turn = true;
                            auto = AutoState.ROTATECW;
                        }
                    } else if (park == 1) {
                        // drive to a bit more on the line
                        drive.driveRobotCentric(0, .47, 0);

                        // if time >= 2100 milliseconds, stop drive motors & reset encoders, switch state
                        // to DONE
                        if (time.milliseconds() >= 2100) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    }
                }

                break;

            // turn to the right
            case ROTATECCW:
                // make sure code only runs once and reset time
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // run if zeroTwo is false (used if there are zero rings on field - set to true in state
                // ROTATECW when zeroOne is true)
                if (!zeroTwo) {
                    // turn towards box B
                    drive.driveRobotCentric(0, 0, -0.45);
                    // if time >= 350 milliseconds, stop driving, set lock to false, and switch to
                    // state DROPWOBBLE
                    if (time.milliseconds() >= 550) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                // run if zeroTwo is true (after ROTATECW when the correct box for the wobble goal
                // is A
                if (zeroTwo) {
                    if (time.milliseconds() >= 200) {
                        // turn right
                        drive.driveRobotCentric(0, 0, -.36);

                        // when time is >= 1000 milliseconds, stop driving, set lock to false, and switch
                        // to state DRIVETOMID
                        if (time.milliseconds() >= 1200) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVETOMID;
                        }
                    }
                }

                break;

            // drive forward and drop the wobble goal in box assigned by visoin code
            case DROPWOBBLE:
                // make sure state only  once - run at beginning of state, reset time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // if time >= 1000 milliseconds and zeroTwo is false, set lock to false, reset time,
                // and switch to state DRIVETOMID
                // if time >= 1000 milliseconds and zeroTwo is true, set lock to false, reset time,
                // and switch to state ROTATECCW
                if(time.milliseconds() >= 1000 && !zeroTwo && !single) {
                    // release servo holding wobble goal
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.DRIVETOMID;
                } else if (time.milliseconds() >= 1000 && zeroTwo) {
                    // release servo holding wobble goal
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECCW;
                } else if (time.milliseconds() >= 1000 & single) {
                    // release servo holding wobble goal
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECW;
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
