package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class CompetitionAuto extends AutoSuperOp {
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
        // if started is false, reset time and set started to true
        if (!started) {
            // reset time and set started to true
            time.reset();
            started = true;
        }

        // add telemetry
        telemetry.addData("state", auto);
        telemetry.addData("time", time);
        telemetry.addData("voltage", getBatteryVoltage());
        telemetry.addData("rings", numRings);
        telemetry.addData("touch sensor pressed?", touchSensor.isPressed());

        // the state machine for this OpMode
        switch(auto) {
            // detect rings
            case DETECT:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // run detect method from AutoSuperOp
                detect();

                // give vision code 2000 milliseconds to run, then switch to state STRAFECW, and
                // reset lock
                if (time.milliseconds() >= 2000) {
                    // reset lock and switch to state STRAFECW
                    lock = false;
                    auto = AutoState.STRAFECW;
                }

                break;

            // strafe towards the left - ran once after state DETECT
            case STRAFECW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // strafe to the left at motor power = .55
                drive.driveRobotCentric(.55, 0, 0);
                // check if time is greater than 1000 milliseconds. if it is, reset lock, stop
                // motors, reset encoders, and switch to state DRIVETOMID
                if (time.milliseconds() >= 1000) {
                    // reset lock, stop motors, reset encoders, and switch to state DRIVETOMID
                    lock = false;
                    resetDrive();
                    auto = AutoState.DRIVETOMID;
                }

                break;

            // drive to right behind midline - runs once, right after state STRAFECW
            case DRIVETOMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // drive forward at motor power = .4
                drive.driveRobotCentric(0,.4, 0);
                // if time is greater that 2200 milliseconds and the battery's voltage is above
                // 13.8, reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                // if time is greater that 2800 milliseconds and the battery's voltage is above
                // 12.6, reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                // if battery is below 12.6 volts and the time is greater than 3400 milliseconds,
                // reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                if(time.milliseconds() >= 2200 && getBatteryVoltage() > 13.8) {
                    // reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                } else if(time.milliseconds() >= 2800 && getBatteryVoltage() > 12.6) {
                    // reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                } else if (time.milliseconds() >= 3400) {
                    // reset lock, stop motors, reset encoders, and switch to state ROTATECWSTART
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                }

                break;

            // rotate clockwise, in this case towards the far field wall - runs once, right after
            // state DRIVETOMID
            case ROTATECWSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // turn to far field wall at motor power = .5
                drive.driveRobotCentric(0,0,.5);
                // check if time is greater than 800 milliseconds. if it is, stop motors, reset
                // encoders, set lock to false, and switch to state DRIVEFORWARDSTART
                if(time.milliseconds() >= 800) {
                    // stop motors, reset encoders, set lock to false, and switch to state DRIVEFORWARDSTART
                    resetDrive();
                    lock = false;
                    auto = AutoState.DRIVEFORWARDSTART;
                }

                break;

            // drive forward to get in position for shooting - used once, right after ROTATECWSTART
            case DRIVEFORWARDSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // drive forward at motor power = .5
                drive.driveRobotCentric(0,.5,0);
                // check if time is greater than 500 milliseconds. if it is, stop motors, reset
                // encoders, set lock to false and switch to state ROTATECCWSTART
                if(time.milliseconds() >= 500) {
                    // stop motors, reset encoders, set lock to false and switch to state ROTATECCWSTART
                    resetDrive();
                    lock = false;
                    auto = AutoState.ROTATECCWSTART;
                }

                break;

            // rotate towards goal so that the shooter is facing goal - used once, right after state
            // DRIVEFORWARDSTART
            case ROTATECCWSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // turn towards goal at motor power = -.5
                drive.driveRobotCentric(0,0,-.5);
                // check if time is greater than 750 milliseconds. if it is, stop motors, reset
                // encoders, reset lock, and switch to state ROTATEWOBBLE
                if(time.milliseconds() >= 750) {
                    // stop motors, reset encoders, reset lock, and switch to state ROTATEWOBBLE
                    resetDrive();
                    lock = false;
                    auto = AutoState.ROTATEWOBBLE;
                }

                break;

            // rotate the teleop wobble arm down from starting position, so that the robot can shoot
            // - used once, right after state ROTATECCWSTART
            case ROTATEWOBBLE:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // set motor wobbleRotate to move down at motor power -.7
                wobbleRotate.set(-.7);
                // check if time is greater than 500 milliseconds. if it is, stop wobbleRotate, reset
                // lock, and switch to state SHOOT
                if(time.milliseconds() >= 500) {
                    // stop wobbleRotate, reset lock, and switch to state SHOOT
                    wobbleRotate.set(0);
                    lock = false;
                    auto = AutoState.SHOOT;
                }

                break;

            // shoot rings into mid or high goal (depends on battery) - used once, right after state
            // ROTATEWOBBLE
            case SHOOT:
                // make sure code only runs once, reset the time, and turn on shooter at motor power
                // = .85
                if (!lock) {
                    lock = true;
                    time.reset();
                    shooter.set(.85);
                }

                // check if the shooter has been running for 2000 milliseconds (time is greater than
                // 2000 milliseconds). if it is, turn on carousel at motor power = 1
                if(time.milliseconds() >= 2000) {
                    // turn on carousel at motor power = 1
                    carousel.set(1);
                }

                // check if time is greater than 4000 milliseconds (enough time to shoot all three
                // rings). if it is, stop carousel, stop shooter, and reset lock. then set wobbleRotate
                // to move up slightly at motor power = .6 (stop after 100 milliseconds). if the
                // number of rings detected (numRings) is 0, switch to state ROTATECCW. else, switch
                // to state DRIVEOVERMID
                if(time.milliseconds() >= 4000) {
                    // stop carousel, stop shooter, and reset lock
                    carousel.set(0);
                    shooter.set(0);
                    lock = false;

                    // turn on wobbleRotate at motor power = .6
                    wobbleRotate.set(.6);
                    // check if time is greater than 100 milliseconds. if it is, stop wobble rotate
                    if(time.milliseconds() >= 100) {
                        // stop wobbleRotate
                        wobbleRotate.set(0);
                    }

                    // if the number of rings on the field is 0 (numRings = 0), then switch to state
                    // ROTATECCW. otherwise, switch to state DRIVEOVERMID
                    if(numRings == 0) {
                        // switch to state ROTATECCW
                        auto = AutoState.ROTATECCW;
                    } else {
                        // switch to state DRIVEOVERMID
                        auto = AutoState.DRIVEOVERMID;
                    }
                }

                break;

            // rotate towards the wall closest to the robot - used after state SHOOT if there are 0
            // rings on the field and after state DRIVEOVERMID if there are 4 rings on the field
            case ROTATECCW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // if there are 0 rings on the field, turn into box A, towards the closest wall
                // until time is greater than 700 milliseconds. then stop motors, reset encoders,
                // reset lock, and switch to state DROPWOBBLE
                // if there are 4 rings on the field, turn into box C, towards the closest wall.
                // if the battery's voltage is greater than 13 volts, turn for 600 seconds, otherwise,
                // turn for 700. when time conditions are met, stop motors, reset encoders, reset
                // lock, and switch to state DRIVEABIT
                if (numRings == 0) {
                    // turn towards box A and closest wall at motor power = -.32
                    drive.driveRobotCentric(0,0,-.32);
                    // check if time is greater than 700 milliseconds. if it is, stop motors, reset
                    // encoders, reset lock, and switch to state DROPWOBBLE
                    if(time.milliseconds() >= 700) {
                        // stop motors, reset encoders, reset lock, and switch to state DROPWOBBLE
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                } else if (numRings == 4) {
                    // turn towards box C and closest wall at motor power = -.32
                    drive.driveRobotCentric(0,0,-.32);
                    // if the time is greater than 600 milliseconds and the voltage is greater than
                    // 13, stop motors, reset encoders, reset lock , and switch to state DRIVEABIT
                    // if the voltage is less than 13, wait until time is greater than 700 milliseconds,
                    // then stop motors, reset encoders, reset lock , and switch to state DRIVEABIT
                    if(time.milliseconds() >= 600 && getBatteryVoltage() > 13) {
                        // stop motors, reset encoders, reset lock , and switch to state DRIVEABIT
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVEABIT;
                    } else if (time.milliseconds() >= 700) {
                        // stop motors, reset encoders, reset lock , and switch to state DRIVEABIT
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVEABIT;
                    }
                }

                break;

            // drive a small bit forward - used after state ROTATECCW if there are 4 rings on the field
            case DRIVEABIT:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // drive forward at motor power = .4
                drive.driveRobotCentric(0,.4,0);
                // check if time is greater than 450 milliseconds. if it is, stop motors, reset
                // encoders, reset lock, and switch to state DROPWOBBLE
                if(time.milliseconds() >= 450) {
                    // stop motors, reset encoders, reset lock , and switch to state DROPWOBBLE
                    resetDrive();
                    lock = false;
                    auto = AutoState.DROPWOBBLE;
                }

                break;

            // drive forward over the midline - used after state SHOOT if there are not 0 rings on the field
            case DRIVEOVERMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // if there is one ring on the field, drive forward for 800 milliseconds. then
                // stop motors, reset encoders, reset lock, and switch to state DROPWOBBLE
                if(numRings == 1) {
                    // drive forward at motor power = .5
                    drive.driveRobotCentric(0,.5,0);
                    // check if time is greater than 800 milliseconds. if it is, stop motors, reset
                    // encoders, reset lock, and switch to state DROPWOBBLE
                    if(time.milliseconds() >= 800) {
                        // stop motors, reset encoders, reset lock, and switch to state DROPWOBBLE
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                // if there are 4 rings on the field, drive forward for 1600 seconds if the voltage
                // is greater than 12.7 and 2000 if its not. then stop motors, reset encoders, reset
                // lock, and switch to state ROTATECCW
                if(numRings == 4) {
                    // drive forward at motor power = .5
                    drive.driveRobotCentric(0,.5,0);
                    // if the time is greater than 1600 milliseconds and the voltage is greater than
                    // 12.7 or the time is greater than 2000 milliseconds then stop motors, reset
                    // encoders, reset lock, and switch to state ROTATECCW
                    if(time.milliseconds() >= 1600 && getBatteryVoltage() >= 12.7) {
                        // stop motors, reset encoders, reset lock, and switch to state ROTATECCW
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECCW;
                    } else if(time.milliseconds() >= 2000) {
                        // stop motors, reset encoders, reset lock, and switch to state ROTATECCW
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECCW;
                    }
                }

                break;

            // drop wobble goal in the correct box - used after state ROTATECCW if there are 0 rings
            // on the field. if there's one ring, used after state DRIVEOVERMID. if there are 4 rings
            // on the field, used after state DRIVEABIT
            case DROPWOBBLE:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // release servo holding wobble goal by setting position to 1
                autoWobble.setPosition(1);
                // if the time is greater than 1000 milliseconds then reset lock and switch to state
                // DRIVEBEHINDMID
                if(time.milliseconds() >= 1000) {
                    // reset lock and switch to state DRIVEBEHINDMID
                    lock = false;
                    auto = AutoState.DRIVEBEHINDMID;
                }

                break;

            // drive behind the midline to parks - used after state DROPWOBBLE
            case DRIVEBEHINDMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                // if there are 0 rings on the field, drive forward for 300 milliseconds and switch
                /// to state DONE
                if(numRings == 0) {
                    // drive forward at motor power = .3
                    drive.driveRobotCentric(0,.3,0);
                    // if the time is greater the 300 milliseconds, stop motors, reset encoders, reset
                    // lock, and switch to state DONE
                    if(time.milliseconds() >= 300) {
                        // stop motors, reset encoders, reset lock, and switch to state DONE
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                // if there is one ring on the field, drive backward for 650 milliseconds and switch
                // to state DONE
                if(numRings == 1) {
                    // drive backwards at motor power = -.4
                    drive.driveRobotCentric(0,-.4,0);
                    // if the time is greater than 650 milliseconds, stop motors, reset encoders, reset
                    // lock, and switch to state DONE
                    if(time.milliseconds() >= 650) {
                        // stop motors, reset encoders, reset lock, and switch to state DONE
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                // if there are 4 rings on the field, drive backward for 2600 milliseconds and switch
                // to state DONE
                if(numRings == 4) {
                    // drive backwards at motor power = -.4
                    drive.driveRobotCentric(0,-.4,0);
                    // if the time is greater than 2600 milliseconds, stop motors, reset encoders, reset
                    // lock, and switch to state DONE
                    if(time.milliseconds() >= 2600) {
                        // stop motors, reset encoders, reset lock, and switch to state DONE
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                break;

            // stops OpMode completely - used after state DRIVEBEHINDMID
            case DONE:
                // call method requestOpModeStop from class OpMode
                requestOpModeStop();

                break;
        }
    }
}
