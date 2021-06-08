 package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class SuperOp extends OpMode {
    // declare class constants
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;
    // constant used for the intake's power
    final double INTAKE_POWER = 1;
    // constant used for the carousel's power while indexing
    final double CAROUSEL_INDEXING_POWER = .15;
    // declare threshold for color sensor data
    final double alphaThreshold = 500;
    // constant used to ensure the color sensor doesn't find any false positives/negatives
    final double ringDetectionTimer = 15.0;
    // allow for carousel to single index (power shots)
    final double SINGLE_INDEX_CAROUSEL_TIME = 31;
    // constant used for the shooter power
    final double SHOOTER_POWER = 1;
    // constant used for the carousel's power while shooting
    final double CAROUSEL_SHOOTING_POWER = .85;
    // constant used to update the current time and get a wait time for carousel activation during
    // shooter()
    final double CAROUSEL_ACTIVATION_TIME = 1000.0;
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare drive motors
    Motor frontLeftDrive;
    Motor frontRightDrive;
    Motor backLeftDrive;
    Motor backRightDrive;
    // declare non-drive motors
    Motor carousel;
    Motor shooter;
    Motor intake;
    Motor wobbleRotate;
    // declare servos
    ServoEx teleopWobble;
    ServoEx autoWobble;

    // declare new ElapsedTime (needed for shooter)
    ElapsedTime time;
    // declare new ColorSensor (needed to index rings)
    ColorSensor colorSensor;
    // declare new TouchSensor (needed to ensure wobbleRotate doesn't slam into camera)
    TouchSensor touchSensor;
    // declare drive constructor
    public MecanumDrive drive;

    public void init() {
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm, 0.9);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, 0.9);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, 1);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm, 1);
        // set drive motors' zero power to brake so the driving is more accurate
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // set motors that need to be reversed due to hardware to inverted
        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        // initialize non-drive motors
        carousel = new Motor(hardwareMap, "carousel", cpr, rpm);
        shooter = new Motor(hardwareMap, "shooter", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        wobbleRotate = new Motor(hardwareMap, "wobbleRotate", cpr, rpm);
        // set non-drive motors' zero power to float instead of brake so the motors don't burn out
        // trying to stop
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize servos
        teleopWobble = new SimpleServo(hardwareMap, "teleopWobble");
        autoWobble = new SimpleServo(hardwareMap, "autoWobble");

        // initialize time constructor
        time = new ElapsedTime();
        // initialize ColorSensor
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        // initialize TouchSensor
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        // initialize drive (so we can drive)
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        drive.setMaxSpeed(1);

        // add drive telemetry
        telemetry.addData("Front Left Power", frontLeftDrive.get());
        telemetry.addData("Front Right Power", frontRightDrive.get());
        telemetry.addData("Back Left Power", backLeftDrive.get());
        telemetry.addData("Back Right Power", backRightDrive.get());
        // add index telemetry
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("carousel", carousel.get());
        telemetry.addData("ring count", ringCount);
        telemetry.addData("wait time", carouselActivationWaitTime);
        // add shooter telemetry
        telemetry.addData("Shooter Power", shooter.motor.getPower());
        telemetry.addData("time", time.milliseconds());
    }

    // drive according to controller inputs from driver's sticks
    public void drive() {
        // set the driver's sticks to correspond with the drive method
        // left stick y = forward/ backward; left stick x = strafing; right stick x = turning
        double strafePower = Math.abs(gamepad1.left_stick_x) < THRESHOLD ? 0 : gamepad1.left_stick_x;
        double forwardPower = Math.abs(gamepad1.left_stick_y) < THRESHOLD ? 0 : gamepad1.left_stick_y;
        double turnPower = Math.abs(gamepad1.right_stick_x) < THRESHOLD ? 0 : gamepad1.right_stick_x;
        // call drive robot centric and set to the values found above
        drive.driveRobotCentric(
                -strafePower * 0.8,
                -forwardPower * 0.8,
                turnPower * 0.7,
                true
        );
    }

    // binds intake to right trigger, reverse intake to left trigger
    // both driver and operator can intake, but driver overrides operator
    public void intake() {
        // set intake to 0 as a default
        intake.set(0);

        // if operator presses right trigger, turn on intake
        // if operator presses left trigger, turn on reverse intake
        if(gamepad2.right_trigger > THRESHOLD) {
            intake.set(INTAKE_POWER);
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intake.set(-INTAKE_POWER);
        }

        // if driver presses right trigger, turn on intake
        // if driver presses left trigger, turn on reverse intake
        if(gamepad1.right_trigger > THRESHOLD) {
            intake.set(INTAKE_POWER);
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intake.set(-INTAKE_POWER);
        }
    }

    // check whether or not carousel is running, ensures that carousel turns off at the correct time
    double lastSeenRing = 0.0;
    // check if color sensor is currently looking at a ring
    boolean currentlyViewingRing = false;
    // declare enum for ring's current placement
    enum RingState {
        FRONT, IN_MIDDLE, BACK, NONE
    }
    // declare the current ring state as none
    RingState ringState = RingState.NONE;
    // keep track of how many rings are currently indexed
    double ringCount = 0;
    // autonomously index rings as they're pulled in by the intake
    public void index() {
        // add telemetry
        telemetry.addLine("running this");
        telemetry.addData("> sensor data: ", colorSensor.alpha());
        telemetry.addData("> sensor threshold: ", alphaThreshold);
        telemetry.addData("> currently viewing ring: ", currentlyViewingRing);
        telemetry.addData("> when ring last saw: ", lastSeenRing);
        telemetry.addData("> Ring State: ", ringState);
        telemetry.addData("> ring count: ", ringCount);
        telemetry.addData("> time: ", time.milliseconds());

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
        } else if(time.milliseconds() - ringDetectionTimer > lastSeenRing) {
            if (lastSeenRing != -1) {
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
                    case IN_MIDDLE:
                        carousel.set(CAROUSEL_INDEXING_POWER);
                }
            } else {
                telemetry.addLine("hey doesn't remember rings");
            }
        } else {
            carousel.set(0);
        }
    }

    // reset index variables
    public void resetIndex() {
        lastSeenRing = -1.0;
        carousel.set(0.0 );
        ringState = RingState.NONE;
    }

    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    // set equal to the current value of driver's left bumper
    boolean left_bumper;
    // set equal to the previous value of driver's right bumper
    boolean prevRB = false;
    // set equal to the current value of driver's right bumper
    boolean right_bumper;
    // keep track of whether or not the shooter is currently running
    boolean isShooterRunning = false;
    // keep track of whether or not manual indexing has been activated by driver's right bumper
    boolean manualCarouselIndex = false;
    // set equal to the current value of driver's a
    boolean gamepad1A;
    // set equal to the previous value of driver's a
    boolean prevG1A = false;
    // keep track of driver's a presses
    boolean buttonIndexingActive = false;
    // set equal to the current value of operator's left stick x
    double prevG2Joystick = 0;
    // set equal to the previous value of operator's left stick x
    double g2Joystick;
    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    void shooterRewrite() {
        // set booleans declared above to their corresponding buttons
        right_bumper = gamepad1.right_bumper;
        left_bumper = gamepad1.left_bumper;
        gamepad1A = gamepad1.a;
        g2Joystick = gamepad2.left_stick_x;
        // add telemetry
        telemetry.addData("> Prev: ", prevLB);
        telemetry.addData("> Bumper: ", left_bumper);
        telemetry.addData("> Is shooter running", isShooterRunning);

        // check if the driver's left bumper is pressed and if it's previous value is false (prevLB)
        // or if the driver's right bumper is pressed and if it's previous value is false (prevRB)
        // if both conditions are met, check if manually indexing and toggle isShooterRunning
        if((left_bumper && !prevLB) || (right_bumper && !prevRB)) {
            // if shooter isn't running, check if driver's left bumper is pressed. if it is, set
            // manualCarouselIndex to false and set an activation time for the carousel
            // if driver's left bumper isn't pressed, check for driver's right bumper. if driver's
            // right bumper is pressed, set manualCarouselIndex to true.
            // while shooter is off, after checking driver's bumpers, toggle isShooterRunning.
            // if neither bumper is pressed, call resetIndex() and set isShooterRunning to false
            if (!isShooterRunning) {
                // if driver's left bumper is pressed, set manualCarouselIndex to false and set
                // an activation time for the carousel
                // if driver's right bumper is pressed, set manualCarouselIndex to true
                if (left_bumper) {
                    // set manualCarouselIndex to false and set an activation time for the carousel
                    manualCarouselIndex = false;
                    carouselActivationWaitTime = time.milliseconds() + CAROUSEL_ACTIVATION_TIME;
                } else if (right_bumper) {
                    // set manualCarouselIndex to true
                    manualCarouselIndex = true;
                }

                // toggle isShooterRunning
                isShooterRunning = true;
            } else {
                // call resetIndex() and set isShooterRunning to false
                resetIndex();
                isShooterRunning = false;
            }
        }

        // if the shooter is running, wait for shooter to spin up to full speed, then run carousel
        // to shoot. if it isn't, turn off shooter and allow for operator to manually index with
        // joystick if desired.
        if (isShooterRunning) {
            // turn on shooter
            shooter.set(SHOOTER_POWER);
            // if the wait time has been completed (time is greater than the wait time) and
            // manualCarouselIndex is false, run carousel, set lastSeenRing to -1, and set ringCount
            // to 0.
            // if manualCarouselIndex is true, allow driver to manually shoot with a.
            if (!manualCarouselIndex && time.milliseconds() > carouselActivationWaitTime) {
                // run carousel, set lastSeenRing to -1, and set ringCount to 0
                carousel.set(CAROUSEL_SHOOTING_POWER);
                lastSeenRing = -1;
                ringCount = 0;
            } else if (manualCarouselIndex) {
                // if driver's a is currently pressed and it's previous value is false, check if
                // button indexing is active. if it is, add the single indexing time to the current
                // activation wait time. if it isn't set wait time and set buttonIndexingActive to true
                // in both cases subtract 1 from the ring count and turn on carousel
                if (gamepad1A && !prevG1A) {
                    // if button indexing is active, add single indexing time to existing wait time
                    // if button indexing isn't active, set wait time and set buttonIndexingActive
                    // to true
                    if (buttonIndexingActive) {
                        // add single indexing time to existing wait time
                        carouselActivationWaitTime += SINGLE_INDEX_CAROUSEL_TIME;
                    } else {
                        // set wait time and set buttonIndexingActive to true
                        carouselActivationWaitTime = time.milliseconds() + SINGLE_INDEX_CAROUSEL_TIME;
                        buttonIndexingActive = true;
                    }
                    // subtract 1 from ringCount and turn on carousel
                    ringCount--;
                    carousel.set(CAROUSEL_SHOOTING_POWER);
                }

                // if the shooter hasn't had enough time to spin up, set buttonIndexingActive to false
                // and set carousel to 0
                if (carouselActivationWaitTime < time.milliseconds()) {
                    // set buttonIndexingActive to false and set carousel to 0
                    buttonIndexingActive = false;
                    carousel.set(0);
                }
            }
        } else {
            // turn off shooter
            shooter.set(0);
            // if operator's left stick x is in use, set carousel to stick's power * the carousel
            // indexing power
            // if it isn't in use, check if the previous value of stick was in use
            // if the previous value is greater than the threshold, turn off carousel and call
            // resetIndex() otherwise, index as normal
            if (Math.abs(g2Joystick) > THRESHOLD) {
                carousel.set(g2Joystick * CAROUSEL_INDEXING_POWER);
            } else {
                if (Math.abs(prevG2Joystick) > THRESHOLD) {
                    carousel.set(0);
                    resetIndex();
                } else {
                    index();
                }
            }
        }

        // add telemetry and set previous values to their corresponding values
        telemetry.addData("Prev Joystick", prevG2Joystick);
        prevLB = left_bumper;
        prevRB = right_bumper;
        prevG1A = gamepad1A;
        prevG2Joystick = g2Joystick;
    }

    // set equal to the previous value of operator's a button
    boolean prevA = false;
    // keep track of whether or not the teleopServo is currently open
    boolean isTeleopServo = false;
    // set equal to the previous value of operator's y button
    boolean prevY = false;
    // keep track of whether or not the autoServo is currently open
    boolean isAutoServo = false;
    // check if wobbleRotate was touching the touch sensor
    boolean wasTouching = false;
    // keep track of touches on sensor
    int i = 0;
    // variable keeping track of arm's power
    double wobbleRotatePower;
    // set equal to current value of operator's right stick y
    double g2RightStickY;
    // bind autoWobble to operator's a, bind teleopWobble to operator's y, and bind wobbleRotate to
    // operator's right stick y (also uses the touch sensor to stop wobbleRotate)
    public void wobbleGoals() {
        // if operator's a button is pressed and it's previous value is false (prevA), check if
        // teleopServo is open (isTeleopServo). if it is, close it and if it isn't, open it. No
        // matter what, toggle isTeleopServo
        if (gamepad2.a && !prevA) {
            // if teleop servo is open, close on a press, otherwise open on a press
            if (isTeleopServo) {
                // if servo is open, close on a press
                teleopWobble.setPosition(.7);
            } else {
                // if servo is closed, open on a press
                teleopWobble.setPosition(-.3);
            }
            // set isTeleopServo to it's opposite (toggle)
            isTeleopServo = !isTeleopServo;
        }
        // set prevA to the current value of operator's a
        prevA = gamepad2.a;

        // if operator's y button is pressed and it's previous value is false (prevY), check if
        // autoServo is open (isAutoServo). if it is, close it and if it isn't, open it. No matter
        // what, toggle isAutoServo
        if (gamepad2.y && !prevY) {
            // if auto servo is open, close on y press, otherwise open on y press
            if (isAutoServo) {
                // if servo is open, close on y press
                autoWobble.setPosition(.7);
            } else {
                // if servo is closed, open on y press
                autoWobble.setPosition(0.1);
            }
            // set isAutoServo to it's opposite (toggle)
            isAutoServo = !isAutoServo;
        }
        // set prevY to the current value of operator's y
        prevY = gamepad2.y;

        // double that allows for neater method code (checks for threshold)
        wobbleRotatePower = 0.0;
        // negate value of operator's right stick y in variable
        g2RightStickY = -gamepad2.right_stick_y;

        // check if the touch sensor is pressed, and if the operator isn't trying to raise arm,
        // raise a small bit
        // if operator is moving arm, check if wasTouching is true. if it is and the operator is
        // moving the arm up, set power to the value of the stick * .5 and set wasTouching to false
        // otherwise, power is set to value of the stick * .5
        if(touchSensor.isPressed()) {
            // if operator's right stick y isn't currently in use, set wobbleRotatePower to .2
            if (g2RightStickY < THRESHOLD) {
                wobbleRotatePower = 0.2;
            }
            // set was Touching to true and increment i by 1
            wasTouching = true;
            i++;
        } else if (Math.abs(g2RightStickY) > THRESHOLD) {
            // if wasTouching is true, check if operator is moving arm up. if they are, allow for
            // arm to be moved and set wasTouching to false
            // else, set power to stick's value * .5
            if (wasTouching) {
                // if operator's right stick y is greater than threshold, set power to the stick's
                // value * .5 and set wasTouching to false
                if (g2RightStickY > THRESHOLD) {
                    // set power to stick's value * .5 and set wasTouching to false
                    wobbleRotatePower = g2RightStickY * 0.5;
                    wasTouching = false;
                }
            } else {
                // set power to stick's value * .5
                wobbleRotatePower = g2RightStickY * 0.5;
            }
        }

        // set arm's power
        wobbleRotate.set(wobbleRotatePower);

        // add telemetry
        telemetry.addData("right stick y: ", g2RightStickY);
        telemetry.addData("raw right stick y: ", gamepad2.right_stick_y);
        telemetry.addData("raw right stick x: ", gamepad2.right_stick_x);
        telemetry.addData("Wobble Rotate Power", wobbleRotatePower);
        telemetry.addData("Touch sensor presses: ", i);
        telemetry.addData("was touching?: ", wasTouching);
        telemetry.addData("is touching: ", touchSensor.isPressed());
    }

    // allows driver and operator to stop all of the hardware they control on dpad down press
    public void emergencyStop() {
        if(gamepad1.dpad_down) {
            // stop drive motors
            frontLeftDrive.set(0);
            frontRightDrive.set(0);
            backLeftDrive.set(0);
            backRightDrive.set(0);

            // stop non-drive motors
            carousel.set(0);
            shooter.set(0);
            intake.set(0);
        }

        if(gamepad2.dpad_down) {
            // stop non-drive motors
            intake.set(0);
            wobbleRotate.set(0);

            // release all servos
            teleopWobble.setPosition(.7);
            autoWobble.setPosition(.7);
        }
    }
}
