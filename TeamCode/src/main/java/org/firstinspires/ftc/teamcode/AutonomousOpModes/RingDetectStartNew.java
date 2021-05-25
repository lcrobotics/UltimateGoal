package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RingDetectStartNew extends AutoSuperOpNew{
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

                drive.driveRobotCentric(.5, 0, 0);
                if (time.milliseconds() >= 900) {
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
                if(time.milliseconds() >= 2900 && getBatteryVoltage() > 12.6) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.STRAFECCW;
                } else if (time.milliseconds() >= 2600) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.STRAFECCW;
                }

                break;

            case STRAFECCW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(-.5, 0, 0);
                if (time.milliseconds() >= 1050) {
                    lock = false;
                    resetDrive();
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
                        auto = AutoState.ROTATECW;
                    } else {
                        auto = AutoState.DRIVEOVERMID;
                    }
                }

                break;

            case ROTATECW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,0,-.32);
                if(time.milliseconds() >= 500) {
                    resetDrive();
                    lock = false;
                    auto = AutoState.DRIVEABIT;
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
                if(time.milliseconds() >= 300) {
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
                    if(time.milliseconds() >= 1400) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECW;
                    }
                }

                if(numRings == 4) {
                    drive.driveRobotCentric(0,.5,0);
                    if(time.milliseconds() >= 2200 && getBatteryVoltage() >= 12.7) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECW;
                    } else if(time.milliseconds() >= 2000) {
                        resetDrive();
                        lock = false;
                        auto = AutoState.ROTATECW;
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
                    if(time.milliseconds() >= 1500) {
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
