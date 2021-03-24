package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class StartAuto extends AutoSuperOp {
    // ensure that TURNABIT actually runs
    boolean started = false;
    // start the OpMode in state SHOOT
    AutoState auto = AutoState.SHOOT;

    public void init() {
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

        telemetry.addData("state", auto);

        switch(auto) {
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
                if (time.milliseconds() >= 4000) {
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
                    auto = AutoState.TURNABIT;
                }

                break;

            case TURNABIT:
                if (!lock) {
                    lock = true;
                    time.reset();
                }

                drive.driveRobotCentric(0,0,.3);
                if (time.milliseconds() >= 500) {
                    resetDrive();
                    lock = false;
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
                    drive.driveRobotCentric(0, -0.45, 0);

                    // if time >= 1700 milliseconds, drive to end up over shooting line
                    // reset encoders and stop drive motors, increment park, switch state to
                    // DROPWOBBLE
                    if (time.milliseconds() >= 3200) {
                        lock = false;
                        resetDrive();
                        park++;
                        auto = AutoState.DROPWOBBLE;
                    }
                } else if (park == 1) {
                    // drive to a bit more on the line
                    drive.driveRobotCentric(0, 0.32, 0);

                    // if time >= 700 milliseconds, stop driv e motors & reset encoders, switch state
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
                drive.driveRobotCentric(0, 0, -0.3);
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
