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
        telemetry.addData("shoot", shoot);
        telemetry.addData("voltage", getBatteryVoltage());
        telemetry.addData("rings", numRings);

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

                drive.driveRobotCentric(.3, 0, 0);
                if (time.milliseconds() >= 400) {
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
                if(time.milliseconds() >= 900) {
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

                drive.driveRobotCentric(-.3, 0, 0);
                if (time.milliseconds() >= 400) {
                    lock = false;
                    resetDrive();
                    auto = AutoState.SHOOT;
                }
                break;

            case SHOOT:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                if(!touchSensor.isPressed()) {
                    wobbleRotate.set(.6);
                }

            case ROTATECW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
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
                // drive a bit forward to ensure wobble lands in box
                break;

            case DRIVEOVERMID:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }
                // drive to box B or box C
                break;

            case ROTATECCW:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }
                // rotate left to make sure wobble gets to right place
                break;

            case DROPWOBBLE:
                // make sure code only runs once and reset the time
                if (!lock) {
                    lock = true;
                    time.reset();
                }
                // drop wobble in correct box
                break;

            case DONE:
                requestOpModeStop();
                break;
        }
    }
}
