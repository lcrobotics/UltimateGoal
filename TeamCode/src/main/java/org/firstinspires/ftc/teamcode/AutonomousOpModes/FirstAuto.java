package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class FirstAuto extends AutoSuperOp {
    // what state of autonomous we are in
    AutoState auto = AutoState.DRIVE;
    // number of attempts to find nav target
    int rotNum = 0;
    ObjectLocator.RobotPos lastPos;
    ElapsedTime time;
    boolean target;
    boolean targetFound = false;
    boolean lock = false;
    int ringsShot = 0;
    @Override
    public void init() {
        super.init();
        time = new ElapsedTime();
    }

    public void loop() {
        switch (auto) {

            // drive from starting position to just past shooting line
            // so that the sensors can see the picture.
            case DRIVE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, -0.5, 0);
                // once robot drives for >3 secs, goes to clockwise case
                // resets lock
                if (time.seconds() >= 2.2) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.CLOCKWISE;
                }
                break;

            case STOPCHECK:

                if (!lock) {
                    drive.stop();
                    time.reset();
                    lock = true;
                }

                if (time.seconds() <= 0.3) {

                    // attempt to get robot's location based on nav target
                    objectLocator.updateRobotLocation();

                    if (objectLocator.targetVisible) {
                        lock = false;
                        targetFound = true;
                        auto = AutoState.ANGLE;
                    }

                } else {
                    lock = false;

                }

                break;

                // Turns robot clockwise to look for picture.
            case CLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // time for rotation
                int rotTime = 1;

                drive.driveRobotCentric(0, 0, 0.2);

                break;

                // if robot passes picture, turns the other way (counterclockwise) to look again.
            case COUNTERCLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, -0.2);

                // attempt to get robot's location based on nav target
                objectLocator.updateRobotLocation();

                // if the target is visible, go to angle case
                if (objectLocator.targetVisible) {
                    lock = false;
                    drive.stop();
                    auto = AutoState.ANGLE;
                // if the robot has not seen picture in 4 secs, go back to clockwise to try again
                } else if (time.seconds() >= 4) {
                    lock = false;
                    rotNum++;
                    auto = AutoState.CLOCKWISE;
                }
                break;

                // get robot's position based on nav target and update angle
            // until the robot is facing the nav target directly
            case ANGLE:

                // optimal angle the robot should be facing
                lastPos = objectLocator.lastPos;

                // adjust position until angle is within pre-decided threshold
                if (lastPos.w > 92) {
                    drive.driveRobotCentric(0, 0, 0.5);
                } else if (lastPos.w < 88) {
                    drive.driveRobotCentric(0, 0, -0.5);
                // if angle is within threshold, go to sideways case
                } else {
                    drive.stop();
                    auto = AutoState.SIDEWAYS;
                }

                break;

                // finds translational delta (only R/L) and adjusts to within threshold.
            case SIDEWAYS:

                // get robot's position and update sideways position
                double desiredY = 300;

                lastPos = objectLocator.lastPos;

                // adjust position until robot pos is within a pre-decided threshold
                if (lastPos.y > desiredY + 2) {
                    drive.driveRobotCentric(0.5, 0, 0);
                } else if (lastPos.y < desiredY - 2) {
                    drive.driveRobotCentric(-0.5, 0, 0);
                // when robot within threshold, go to back case
                } else {
                    auto = AutoState.BACK;
                }

                // finds distance robot needs to go to be behind shooting line and adjusts.
            case BACK:

                // get robot's position and go back
                double desiredX = 500;

                lastPos = objectLocator.lastPos;

                // adjust robot position based on pre-determined values
                if (lastPos.x > desiredX + 2) {
                    drive.driveRobotCentric(0, -0.5, 0);
                } else if (lastPos.x < desiredX - 2) {
                    drive.driveRobotCentric(0, 0.5, 0);
                // if robot in right place, go to shoot case
                } else {
                    auto = AutoState.SHOOT;
                }

                // shoots ring into goal
            case SHOOT:

                if (!lock) {
                    // sets shooter motor to 1, will run for rest of case
                    shooter.set(1);
                    time.reset();
                    lock = true;
                    target = true;
                }


                if (time.seconds() > 0.5) {

                    // set servo to 0 = second half of shooting action
                    shooterServo.setPosition(target? 0 : 1);
                    time.reset();
                    target = !target;
                    ringsShot++;

                } else {
                    // set servo to 1 = first half of shooting action
                    shooterServo.setPosition(target? 1 : 0);
                }
                // caps the number of shots at 3
                if (ringsShot == 6) {
                    lock = false;
                    auto = AutoState.FAIL;
                }
                break;

                // stops OpMode
            case FAIL:
                requestOpModeStop();
                break;
        }
    }
}