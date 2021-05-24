package org.firstinspires.ftc.teamcode.TeleopOpModes;

import android.media.Ringtone;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PeytonsSensor extends OpMode {
    // declare class constants
    // constant used to update the current time and get a wait time for carousel activation
    final double CAROUSEL_ACTIVATION_TIME = 1000.0;
    // constant used for the shooter power
    final double SHOOTER_POWER = 1;
    // constant used for the carousel's power while shooting
    final double CAROUSEL_SHOOTING_POWER = 1;
    // constant used for the intake's power
    final double INTAKE_POWER = 1;
    // constant used for the carousel's power while indexing
    final double CAROUSEL_INDEXING_POWER = .2;
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;

    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare motors
    Motor carousel;
    Motor shoot;
    Motor intake;

    // declare new ElapsedTime (needed for shooter)
    ElapsedTime time;
    // declare new ColorSensor (needed to index rings)
    ColorSensor colorSensor;

    public void init() {
        // initialize motors
        carousel = new Motor(hardwareMap, "carousel", cpr, rpm);
        shoot = new Motor(hardwareMap, "shooter", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize time constructor
        time = new ElapsedTime();
        // initialize ColorSensor
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
    }

    @Override
    public void loop() {
        // call TeleOp methods
//        newShooter();
//        newIntake();
        index();

    }


    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    // shooter booleans (for toggle)
    boolean isShooterRunning = false;
    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    // ensure that the carousel's wait time is only set once per toggle
    boolean isCarouselWaitSet = false;
    boolean isCarouselManuallyStopped = false;
    // run shoot on driver's left bumper press and run spin a second later
    public void newShooter() {
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        isCarouselManuallyStopped = false;
        // check if the driver's left bumper is pressed and if it's previous value is false (prevLB)
        // if both conditions are met, toggle boolean isShooterRunning, causing the shooter to either
        // turn on or off
        if(gamepad1.left_bumper && !prevLB) {
            // toggle boolean isShooterRunning, causing the shooter to either turn on or off
            isShooterRunning = !isShooterRunning;
        }

        // if isShooterRunning is false, stop both motors and reset isCarouselWaitSet back to false
        if(!isShooterRunning) {
            // stop both motors and reset isCarouselWaitSet back to false
            carousel.set(0);
            shoot.set(0);
            isCarouselWaitSet = false;
        }

        // if isShooterRunning is true, turn on shooter
        if(isShooterRunning) {
            // turn on shooter
            shoot.set(SHOOTER_POWER);
        }

        if (gamepad1.right_bumper) {
            // if driver presses right bumper, turn on carousel (used just in case rings get stuck in position)
            if (gamepad1.right_bumper) {
                carousel.set(CAROUSEL_SHOOTING_POWER);
            } else {
                carousel.set(0);
            }
            isCarouselManuallyStopped = true;
        } else if (!isCarouselManuallyStopped) {
            // if isCarouselWaitSet (initialized as false) is false and isShooterRunning is true set the
            // carouselActivationWaitTime to the current time + CAROUSEL_ACTIVATION_TIME (set to 1000),
            // and set isCarouselWaitSet to true, ensuring the activation time is only set once per toggle
            if (!isCarouselWaitSet && isShooterRunning) {
                // set carousel wait time to now + 1000 milliseconds (value of constant
                // CAROUSEL_ACTIVATION_TIME) and set isCarouselWaitSet to true
                carouselActivationWaitTime = time.milliseconds() + CAROUSEL_ACTIVATION_TIME;
                isCarouselWaitSet = true;
            }

            // if the wait time has been completed (time is greater than the wait time) and the wait time
            // has already been set, run carousel
            if (time.milliseconds() > carouselActivationWaitTime && isCarouselWaitSet) {
                // run carousel
                carousel.set(CAROUSEL_SHOOTING_POWER);
                ringCount = 0;
            }
        }
        // set prevLB to the driver's left bumper
        prevLB = gamepad1.left_bumper;
    }

    // takes care of motors pertaining to the intake (intake, reverse intake, and spin)
    public void newIntake () {
        // if driver presses right trigger, turn on intake
        // if driver presses left trigger, reverse intake (useful if rings get stuck somehow)
        if(gamepad1.right_trigger > THRESHOLD) {
            intake.set(INTAKE_POWER);
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intake.set(-INTAKE_POWER);
        } else {
            intake.set(0);
        }
    }


    // declare threshold for color sensor data
    double alphaThreshold = 500;
    // check whether or not carousel is running, ensures that carousel turns off at the correct time
    boolean isCarouselRunning = false;

    double lastSeenRing = 0.0;
    double ringDetectionTimer = 15.0;
    boolean currentlyViewingRing = false;
    enum RingState {
        FRONT, IN_MIDDLE, BACK, NONE
    }
    RingState ringState = RingState.NONE;

    double ringCount = 0;

    // autonomously index rings as they're pulled in by the intake
    public void index() {
        telemetry.addData("> sensor data: ", colorSensor.alpha());
        telemetry.addData("> sensor threshold: ", alphaThreshold);
        telemetry.addData("> currently viewing ring: ", currentlyViewingRing);
        telemetry.addData("> when ring last saw: ", lastSeenRing);
        telemetry.addData("> Ring State: ", ringState);

        telemetry.addData("> ring count: ", ringCount);
        telemetry.addData("> carousel on: ", isCarouselRunning);
        telemetry.addData("> time: ", time.milliseconds());
        telemetry.addData("> Ring Detection Time Threshold: ", ringDetectionTimer);




        // TODO: sync into one method
//        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // TODO: figure out where ringCount can be manipulated elsewhere

        // determines whether the sensor can detect a ring
        currentlyViewingRing = alphaThreshold < colorSensor.alpha();

        // An observed ring only needs to be modified when it was not previously seen
        if (currentlyViewingRing) {
            // if the ringState previously indicates that the ring was not visible, increment state
            switch (ringState) {
                // if a new ring is seen, start carousel to move ring
                case NONE:
                    // the third ring we want to not be indexed, so we only increase count but not move on to the indexing states
                    if (ringCount < 3) {
                        ringCount++;

                        // the first two rings we want to index, so we increment state and set carousel motor power
                        if (ringCount <= 2) {
                            ringState = RingState.FRONT;
                            carousel.set(CAROUSEL_INDEXING_POWER);
                        }
                    }
                    break;
                case IN_MIDDLE:
                    ringState = RingState.BACK;
                    break;
            }
            // start a timer to monitor when the last time it was seen
            // this deals with false negatives given by hardware
            lastSeenRing = time.milliseconds();
        // if the visibility of the ring has timed out, increment the visible states
        } else if(time.milliseconds() - ringDetectionTimer > lastSeenRing && lastSeenRing != -1) {
            switch (ringState) {
                case FRONT:
                    ringState = RingState.IN_MIDDLE;
                    break;
                // if the ring state was the back, we can stop the carousel and increase the ring count on the carousel
                case BACK:
                    ringState = RingState.NONE;
                    carousel.set(0);
                    lastSeenRing = -1;
                    break;
            }
        }
        telemetry.addData("> Timeout? ", time.milliseconds() - ringDetectionTimer > lastSeenRing);
    }
}
