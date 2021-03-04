package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auto extends AutoSuperOp {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    // what state of autonomous we are in
    AutoState auto = AutoState.DRIVEOVERMID;
    // number of attempts to find nav servoPos
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
            case DRIVEOVERMID:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, -0.5, 0);
                // once robot drives for >3 secs, goes to clockwise case
                // resets lock
                if (time.seconds() >= 3) {
                    lock = false;
                    auto = AutoState.ROTATECW;
                }
                break;

            // Turns robot clockwise to look for picture.
            case ROTATECW:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                // select time for rotation based on the turnCount we are on
                int rotTime = 4;
                // if number of rotation = 0 or 4, rotation time = 2 seconds
                // if not, rotation time = 4 seconds
                if (rotNum == 0 || rotNum == 4) {
                    rotTime = 2;
                }

                drive.driveRobotCentric(0, 0, 0.5);

                // attempt to get robot's location based on nav servoPos
                objectLocator.updateRobotLocation();
                if (objectLocator.targetVisible) {
                    lock = false;
                    auto = AutoState.FIXANGLE;
                    // if time is < rotation time, reset lock and go to DONE
                } else if (time.seconds() >= rotTime) {
                    lock = false;
                    if (rotNum == 4) {
                        auto = AutoState.DONE;
                        // if servoPos is not visible, go to counterclockwise
                    } else {
                        auto = AutoState.ROTATECCW;
                        rotNum++;
                    }
                }
                break;

            // if robot passes picture, turns the other way (counterclockwise) to look again.
            case ROTATECCW:

                if (!lock) {
                    time.reset();
                    lock = true;
                }

                drive.driveRobotCentric(0, 0, -0.5);

                // attempt to get robot's location based on nav servoPos
                objectLocator.updateRobotLocation();

                // if the servoPos is visible, go to angle case
                if (objectLocator.targetVisible) {
                    lock = false;
                    auto = AutoState.FIXANGLE;
                    // if the robot has not seen picture in 4 secs, go back to clockwise to try again
                } else if (time.seconds() >= 4) {
                    lock = false;
                    rotNum++;
                    auto = AutoState.ROTATECW;
                }
                break;

            // get robot's position based on nav servoPos and update angle
            // until the robot is facing the nav servoPos directly
            case FIXANGLE:

                // optimal angle the robot should be facing
                lastPos = objectLocator.lastPos;

                // adjust position until angle is within pre-decided threshold
                if (lastPos.w > 92) {
                    drive.driveRobotCentric(0, 0, 0.5);
                } else if (lastPos.w < 88) {
                    drive.driveRobotCentric(0, 0, -0.5);
                    // if angle is within threshold, go to sideways case
                } else {
                    auto = AutoState.STRAFETOTARGET;
                }

                break;

            // finds translational delta (only R/L) and adjusts to within threshold.
            case STRAFETOTARGET:

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
                    auto = AutoState.CORRECTX;
                }

                // finds distance robot needs to go to be behind shooting line and adjusts.
            case CORRECTX:

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
                    auto = AutoState.DONE;
                }
                break;

            // stops OpMode
            case DONE:
                requestOpModeStop();
                break;
        }
        telemetry.update();
    }
}
