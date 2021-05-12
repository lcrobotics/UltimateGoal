package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
 * Auto OpMode that uses ring detection to inform the robot's pathing
 *
 * BASIC PATHING: at start, detect rings. Drive forward a small bit, shoot preloaded rings.
 * turn away from rings in center and drive forward (length of time based on number of rings and
 * robot's voltage). Turn towards correct box for wobble goal (determined by the rings on the field) and
 * drop the wobble goal. Turn to avoid hitting the goal/walls, and drive to park over the shooting line.
 */

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
                    // drive forward at -.35 power
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

            // runs at the beginning of the OpMode, after state SHOOT and runs after states
            // DRIVEOVERMID and DROPWOBBLE, depending on the number of rings on the field. turns
            // clockwise based on the robot, in this OpMode clockwise is to the left
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
                        // turn to the left at -.34 power
                        drive.driveRobotCentric(0,0,-.34);
                        // wait 300 milliseconds then stop motors, reset encoders, set lock to false
                        // and switch to state DRIVEOVERMID
                        if (time.milliseconds() >= 300) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVEOVERMID;
                        }
                    } else {
                        // turn to the left at -.34 power
                        drive.driveRobotCentric(0,0,-.34);
                        // wait 500 milliseconds then stop motors, reset encoders, set lock to false,
                        // and switch to state DRIVEOVERMID
                        if (time.milliseconds() >= 500) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVEOVERMID;
                        }
                    }
                }

                // if rotateZeroCW is true (set to true when there are zero rings on the field in
                // state DRIVEOVERMID) turn to the left for 675 milliseconds then stop motors, reset
                // encoders, set lock to false, set rotateZeroCCW to true (needed in state ROTATECCW),
                // and switch to state DROPWOBBLE
                if(rotateZeroCW) {
                    // turn to the left at -.32 power
                    drive.driveRobotCentric(0,0,-.32);
                    // wait 675 milliseconds then stop motors, reset encoders, set lock to false,
                    // set rotateZeroCCW to true, and switch to state DROPWOBBLE
                    if (time.milliseconds() >= 675) {
                        resetDrive();
                        lock = false;
                        rotateZeroCCW = true;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                // if rotateSingle is true (set to true when there is one ring on the field in state
                // DRIVEOVERMID) wait 300 milliseconds (to let the wobble goal fall) then turn left
                // for 600 milliseconds. After 600 milliseconds, stop motors, reset encoders, and wait
                // 300 more milliseconds (wobble can tip into robot instead of falling over when
                // the robot pauses), then set lock to false, and switch to state DRIVETOMID
                if (rotateSingle) {
                    // wait for 300 milliseconds to allow wobble goal time to fall
                    if (time.milliseconds() >= 300) {
                        // turn to the left at -.32 power
                        drive.driveRobotCentric(0, 0, -.32);
                        // wait 600 milliseconds then stop motors and reset encoders
                        if (time.milliseconds() >= 900) {
                            resetDrive();
                            // wait 300 more milliseconds (to allow wobble goal to stabilize)then
                            // set lock to false and switch to state DRIVETOMID
                            if (time.milliseconds() >= 1200) {
                                lock = false;
                                auto = AutoState.DRIVETOMID;
                            }
                        }
                    }
                }

                // if rotateQuad is true (set to true when there are four rings on the field in state
                // DRIVEOVERMID) wait 300 milliseconds (to control wobble goal's fall) then check if
                // the battery's voltage is greater than 12.5. if it is turn left for 800
                // milliseconds, if it isn't turn left for 1000 milliseconds. After turning for the
                // correct time motors, reset encoders, set lock to false, and if battery >= 12.5
                // switch to state DRIVEABIT, if battery < 12.5 switch to state DROPWOBBLE
                if(rotateQuad) {
                    // wait 300 milliseconds (to control wobble goal's fall)
                    if (time.milliseconds() >= 300) {
                        // turn to the left at -.4 power
                        drive.driveRobotCentric(0,0,-.4);
                        // check if the battery's voltage is greater than 12.5. if it is turn left for 800
                        // milliseconds, if it isn't turn left for 1000 milliseconds. After turning for the
                        // correct time motors, reset encoders, set lock to false, and if battery >= 12.5
                        // switch to state DRIVEABIT, if battery < 12.5 switch to state DROPWOBBLE
                        if (getBatteryVoltage() >= 12.5) {
                            // wait for 800 milliseconds then stop motors, reset encoders, set lock
                            // to false, and switch to state DRIVEABIT
                            if (time.milliseconds() >= 1100) {
                                resetDrive();
                                lock = false;
                                auto = AutoState.DRIVEABIT;
                            }
                        } else {
                            // wait for 1000 milliseconds then stop motors, reset encoders, set lock
                            // to false, and switch to state DROPWOBBLE
                            if (time.milliseconds() >= 1300) {
                                resetDrive();
                                lock = false;
                                auto = AutoState.DROPWOBBLE;
                            }
                        }
                    }
                }

                break;

            // runs once per OpMode call, right after the first time in state ROTATECW. drives to
            // the correct wobble goal box, based on ring detection
            case DRIVEOVERMID:
                // ensure that DRIVEOVERMID runs once per time in the state (runs at the end of the
                // state, resets time for the next state)
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if there are zero rings on the field, drive forward for 2600 milliseconds, stop
                // motors, reset encoders, set lock to false, set rotateZeroCW to true, and switch
                // to state ROTATECW
                if (numRings == 0) {
                    // drive forward at -.42 power
                    drive.driveRobotCentric(0,-.42,0);
                    // wait 2600 milliseconds then stop motors, reset encoders, set lock to false,
                    // set rotateZeroCW to true, and switch to state ROTATECW
                    if(time.milliseconds() >= 2600) {
                        lock = false;
                        resetDrive();
                        rotateZeroCW = true;
                        auto = AutoState.ROTATECW;
                    }
                }

                // if there is one ring on the field, check if the battery's voltage is greater than
                // 12.6. if it is, drive forward for 3200 milliseconds. if it isn't, drive forward
                // for 3400 milliseconds. After driving, stop motors, reset encoders, set lock to
                // false, and set rotateSingle to true. if voltage >= 12.6, switch to state DROPWOBBLE.
                // if voltage < 12.6, switch to state ROTATECCW
                if (numRings == 1) {
                    // drive forward at -.48 power
                    drive.driveRobotCentric(0, -0.48, 0);
                    // check if the battery's voltage is greater than 12.6. if it is, drive forward
                    // for 3200 milliseconds. if it isn't, drive forward for 3400 milliseconds. After
                    // driving, stop motors, reset encoders, set lock to false, and set rotateSingle
                    // to true. if voltage >= 12.6, switch to state DROPWOBBLE. if voltage < 12.6,
                    // switch to state ROTATECCW
                    if (getBatteryVoltage() >= 12.6) {
                        // wait for 3200 milliseconds then stop motors, reset encoders, set lock to
                        // false, set rotateSingle to true, and switch to state DROPWOBBLE
                        if (time.milliseconds() >= 3200) {
                            lock = false;
                            rotateSingle = true;
                            resetDrive();
                            auto = AutoState.DROPWOBBLE;
                        }
                    } else {
                        // wait for 3400 milliseconds then stop motors, reset encoders, set lock to
                        // false, set rotateSingle to true, and switch to state ROTATECCW
                        if (time.milliseconds() >= 3400) {
                            lock = false;
                            rotateSingle = true;
                            resetDrive();
                            auto = AutoState.ROTATECCW;
                        }
                    }
                }

                // if there are four rings on the field, check if battery voltage is greater than
                // 12.5. if it is, drive forward for 3600 milliseconds. if it isn't drive forward for
                // 3800 milliseconds. After driving, stop motors, reset encoders, set lock to false,
                // set rotateQuad to true, and switch to state ROTATECW
                if (numRings == 4) {
                    // drive forward at -.53 power
                    drive.driveRobotCentric(0, -.53, 0);
                    // check if battery voltage is greater than 12.5. if it is, drive forward for
                    // 3600 milliseconds. if it isn't drive forward for3800 milliseconds. After
                    // driving, stop motors, reset encoders, set lock to false, set rotateQuad to
                    // true, and switch to state ROTATECW
                    if (getBatteryVoltage() >= 12.5) {
                        // wait 3600 milliseconds then stop motors, reset encoders, set lock to
                        // false, set rotateQuad to true, and switch to state ROTATECW
                        if (time.milliseconds() >= 3600) {
                            lock = false;
                            resetDrive();
                            rotateQuad = true;
                            auto = AutoState.ROTATECW;
                        }
                    } else {
                        // wait 3800 milliseconds then stop motors, reset encoders, set lock to
                        // false, set rotateQuad to true, and switch to state ROTATECW
                        if (time.milliseconds() >= 3800) {
                            lock = false;
                            resetDrive();
                            rotateQuad = true;
                            auto = AutoState.ROTATECW;
                        }
                    }
                }

                break;

            // runs after some instances of states DRIVEOVERMID and DROPWOBBLE, depending on the
            // number of rings on the field. turns counterclockwise based on the robot, in this OpMode
            // counterclockwise is to the right
            case ROTATECCW:
                // ensure that ROTATECCW runs once per time in the state (runs at the end of the
                // state, resets time for the next state)
                if(!lock) {
                    lock = true;
                    time.reset();
                }

                // if rotateZeroCCW is true (set to true in state ROTATECW when there are zero rings
                // on the field) pause for 200 milliseconds (to allow wobble goal to stabilize by
                // hitting the robot) then turn right for 1100 milliseconds. After turning stop motors,
                // reset encoders, set lock to false, and switch to state DRIVETOMID
                if (rotateZeroCCW) {
                    // wait 200 milliseconds (to allow wobble goal to stabilize by hitting the robot)
                    if (time.milliseconds() >= 200) {
                        // turn to the right at .36 power
                        drive.driveRobotCentric(0, 0, .36);
                        // wait for 1100 milliseconds then stop motors, reset encoders, set lock to
                        // false, and switch to state DRIVETOMID
                        if (time.milliseconds() >= 1300) {
                            resetDrive();
                            lock = false;
                            auto = AutoState.DRIVETOMID;
                        }
                    }
                }

                // if rotateSingle is true (set to true in state DRIVEOVERMID when there is one ring
                // on the field) turn right for 300 milliseconds, then stop motors, reset encoders,
                // set lock to false, and switch to state DROPWOBBLE
                if (rotateSingle) {
                    // turn to the right at .45 power
                    drive.driveRobotCentric(0, 0, 0.45);
                    // wait 300 milliseconds then stops motors, reset encoders, set lock to false,
                    // and switch to state DROPWOBBLE
                    if (time.milliseconds() >= 300) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                // if rotateQuad is true (set to true in state DRIVEOVERMID when there are four rings
                // on the field) turn right for 700 milliseconds then stop motors, reset encoders,
                // set lock to false, and switch to state DRIVETOMID
                if (rotateQuad) {
                    // turn to the right at .36 power
                    drive.driveRobotCentric(0,0, .36);
                    // wait 700 milliseconds then stop motors, reset encoders, set lock to false, and
                    // switch to state DRIVETOMID
                    if(time.milliseconds() >= 700) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVETOMID;
                    }
                }

                break;

            // runs after states DRIVEABIT, ROTATECW, DRIVEOVERMID, and ROTATECCW depending on the
            // number of rings on the field. drops wobble goal in correct square
            case DROPWOBBLE:
                // ensure that DROPWOBBLE runs once per time in the state (runs at the end of the
                // state, resets time for the next state)
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // if time is greater than 1000 milliseconds and either rotateZeroCCW (set to true in
                // state ROTATECW when there are zero rings on the field) or rotateQuad (set to true
                // in state DRIVEOVERMID when there are four rings on the field) are true, release
                // wobble goal, reset time, set lock to false, and switch to state ROTATECCW
                if(time.milliseconds() >= 1000 && (rotateZeroCCW || rotateQuad)) {
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECCW;
                }

                // if time is greater than 1000 milliseconds and rotateSingle (set to true in state
                // DRIVEOVERMID when there is one ring on the field) is true, release wobble goal,
                // reset time, set lock to false, and switch to state ROTATECW
                if (time.milliseconds() >= 1000 && rotateSingle) {
                    topHook.setPosition(0);
                    lock = false;
                    time.reset();
                    auto = AutoState.ROTATECW;
                }

                break;

            // runs after states ROTATECW and ROTATECCW, drives to midline and parks
            case DRIVETOMID:
                // ensure that DRIVETOMID runs once per time in the state (runs at the end of the
                // state, resets time for the next state)
                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // if there are zero rings on the field, drive forward for 1050 milliseconds. After
                // driving, stop motors, reset encoders, set lock to false, and switch to state DONE
                if (numRings == 0) {
                    // drive forward at -.4 power
                    drive.driveRobotCentric(0, -0.4, 0);
                    // wait 1050 milliseconds then stop motors, reset encoders, set lock to false,
                    // and switch to state DONE
                    if(time.milliseconds() >= 1050) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                // if there is one ring on the field, drive backward for 950 milliseconds then stop
                // motors, reset encoders, set lock to false, and switch to state DONE
                if (numRings == 1) {
                    // drive backward at .35 power
                    drive.driveRobotCentric(0, 0.35, 0);
                    // wait 950 milliseconds then stop motors, reset encoders, set lock to false,
                    // and switch to state DONE
                    if (time.milliseconds() >= 950) {
                        lock = false;
                        resetDrive();
                        auto = AutoState.DONE;
                    }
                }

                // if there are four rings on the field, check if the voltage is greater than 12.5.
                // if it is, drive backward for 1000 milliseconds. if not, drive backward for 1400
                // milliseconds. After driving, stop motors, reset encoders, set lock to false, and
                // switch to state DONE
                if (numRings == 4) {
                    // check if the voltage is greater than 12.5. if it is, drive backward for 1000
                    // milliseconds. if not, drive backward for 1400 milliseconds. After driving,
                    // stop motors, reset encoders, set lock to false, and switch to state DONE
                    if (getBatteryVoltage() >= 12.5) {
                        // drive backward at .45 power
                        drive.driveRobotCentric(0, .45, 0);
                        // wait 1000 milliseconds then stop motors, reset encoders, set lock to
                        // false, and switch to state DONE
                        if (time.milliseconds() >= 1000) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    } else {
                        // drive backward at .6 power
                        drive.driveRobotCentric(0, .6, 0);
                        // wait 1400 milliseconds then stop motors, reset encoders, set lock to
                        // false, and switch to state DONE
                        if (time.milliseconds() >= 1400) {
                            lock = false;
                            resetDrive();
                            auto = AutoState.DONE;
                        }
                    }
                }

                break;

            // runs after DRIVETOMID and is the final state of the OpMode. stops OpMode, not just
            // the motors
            case DONE:
                // call OpModeStop from First's OpMode class
                requestOpModeStop();

                break;
        }
    }
}