package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RingDetectStartNew extends AutoSuperOpNew {
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
        if (!started) {
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

                // give vision code 2000 milliseconds to run, then switch to state DRIVEABIT, and
                // reset lock
                if (time.milliseconds() >= 2000) {
                    lock = false;
                    auto = AutoState.STRAFECW;
                }

                break;

            case STRAFECW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(.55, 0, 0);
                if (time.milliseconds() >= 1000) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.DRIVETOMID;
                }

                break;

            case DRIVETOMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,.4, 0);
                if(time.milliseconds() >= 2200 && getBatteryVoltage() > 13.8) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                } else if(time.milliseconds() >= 2800 && getBatteryVoltage() > 12.6) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                } else if (time.milliseconds() >= 3400) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.ROTATECWSTART;
                }

                break;

            case ROTATECWSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,0,.5);
                if(time.milliseconds() >= 800) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.DRIVEFORWARDSTART;
                }

                break;

            case DRIVEFORWARDSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,.5,0);
                if(time.milliseconds() >= 500) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.ROTATECCWSTART;
                }

                break;

            case ROTATECCWSTART:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,0,-.5);
                if(time.milliseconds() >= 750) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.ROTATEWOBBLE;
                }

                break;

            case ROTATEWOBBLE:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                wobbleRotate.set(-.7);
                if(time.milliseconds() >= 500) {
                   wobbleRotate.set(0);
                   lock = false;
                   auto = AutoState.SHOOT;
                }

                break;

            case SHOOT:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                    shooter.set(1);
                }

                if(time.milliseconds() >= 2000) {
                    carousel.set(1);
                }

                if(time.milliseconds() >= 4000) {
                    carousel.set(0);
                    shooter.set(0);
                    lock = false;

                    wobbleRotate.set(.6);
                    if(time.milliseconds() >= 100) {
                        wobbleRotate.set(0);
                    }

                    if(numRings == 0) {
                        auto = AutoState.ROTATECCW;
                    } else {
                        auto = AutoState.DRIVEOVERMID;
                    }
                }

                break;

            case ROTATECCW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }


                if (numRings == 0) {
                    drive.driveRobotCentric(0,0,-.32);
                    if(time.milliseconds() >= 700) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                } else if (numRings == 4) {
                    drive.driveRobotCentric(0,0,-.32);
                    if(time.milliseconds() >= 600 && getBatteryVoltage() > 13) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVEABIT;
                    } else if (time.milliseconds() >= 700) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DRIVEABIT;
                    }
                }
                // rotate and drop wobble in box A (only if 0 rings)
                // rotate and drop wobble in box C (only if 4 rings)
                break;

            case DRIVEABIT:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }
                drive.driveRobotCentric(0,.4,0);
                if(time.milliseconds() >= 450) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.DROPWOBBLE;
                }
                // drive a bit forward to ensure wobble lands in box
                break;

            case DRIVEOVERMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                if(numRings == 1) {
                    drive.driveRobotCentric(0,.5,0);
                    if(time.milliseconds() >= 800) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DROPWOBBLE;
                    }
                }

                if(numRings == 4) {
                    drive.driveRobotCentric(0,.5,0);
                    if(time.milliseconds() >= 1600 && getBatteryVoltage() >= 12.7) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECCW;
                    } else if(time.milliseconds() >= 2000) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECCW;
                    }
                }

                // drive to box B or box C
                break;

            case DROPWOBBLE:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }
                autoWobble.setPosition(1);
                if(time.milliseconds() >= 1000) {
                    lock = false;
                    auto = AutoState.DRIVEBEHINDMID;
                }
                // drop wobble in correct box
                break;

            case DRIVEBEHINDMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                if(numRings == 0) {
                    drive.driveRobotCentric(0,.3,0);
                    if(time.milliseconds() >= 300) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                if(numRings == 1) {
                    drive.driveRobotCentric(0,-.4,0);
                    if(time.milliseconds() >= 650) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                if(numRings == 4) {
                    drive.driveRobotCentric(0,-.4,0);
                    if(time.milliseconds() >= 2400) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.DONE;
                    }
                }

                break;

            case DONE:
                requestOpModeStop();
                break;
        }
    }
}
