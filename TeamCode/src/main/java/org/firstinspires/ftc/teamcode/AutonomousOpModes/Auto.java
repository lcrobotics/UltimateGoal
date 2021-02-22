package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SuperOp;

@Autonomous
public class Auto extends SuperOp {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    // what state of autonomous we are in
    AutoState auto = AutoState.DRIVE;
    // number of attempts to find nav target
    int rotNum = 0;
    ObjectLocator.RobotPos lastPos;
    ElapsedTime time;
    boolean target;
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

                drive.driveRobotCentric(0, 0.5, 0);
                // once robot drives for >3 secs, goes to clockwise case
                // resets lock
                if (time.seconds() >= 3) {
                    lock = false;
                    auto = AutoState.CLOCKWISE;
                }
                break;

            // Turns robot clockwise to look for picture.
            case CLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // select time for rotation based on the rotNum we are on
                int rotTime = 4;
                // if number of rotation = 0 or 4, rotation time = 2 seconds
                // if not, rotation time = 4 seconds
                if (rotNum == 0 || rotNum == 4) {
                    rotTime = 2;
                }

                drive.driveRobotCentric(0, 0, 0.5);

                // attempt to get robot's location based on nav target
                objectLocator.updateRobotLocation();
                if (objectLocator.targetVisible) {
                    lock = false;
                    auto = AutoState.ANGLE;
                    // if time is < rotation time, reset lock and go to FAIL
                } else if (time.seconds() >= rotTime) {
                    lock = false;
                    if (rotNum == 4) {
                        auto = AutoState.FAIL;
                        // if target is not visible, go to counterclockwise
                    } else {
                        auto = AutoState.COUNTERCLOCKWISE;
                        rotNum++;
                    }
                }
                break;

            // if robot passes picture, turns the other way (counterclockwise) to look again.
            case COUNTERCLOCKWISE:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, -0.5);

                // attempt to get robot's location based on nav target
                objectLocator.updateRobotLocation();

                // if the target is visible, go to angle case
                if (objectLocator.targetVisible) {
                    lock = false;
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
