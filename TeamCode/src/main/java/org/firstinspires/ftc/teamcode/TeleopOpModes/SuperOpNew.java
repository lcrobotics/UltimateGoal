 package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class SuperOpNew extends OpMode {
    // declare class constants
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;
    // constant used for the intake's power
    final double INTAKE_POWER = 1;
    // constant used for the carousel's power while indexing
    final double CAROUSEL_INDEXING_POWER = .2;
    // constant used for the shooter power
    final double SHOOTER_POWER = 1;
    // constant used for the carousel's power while shooting
    final double CAROUSEL_SHOOTING_POWER = 1;
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
//        telemetry.addData("manual carousel", isCarouselManuallyStopped);
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
    // both driver and operator can intake, but driver has precedence
    public void intake() {
        intake.set(0);

        if(gamepad2.right_trigger > THRESHOLD) {
            intake.set(INTAKE_POWER);
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intake.set(-INTAKE_POWER);
        }

        if(gamepad1.right_trigger > THRESHOLD) {
            intake.set(INTAKE_POWER);
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intake.set(-INTAKE_POWER);
        }
    }





    // declare threshold for color sensor data
    final double alphaThreshold = 500;
    // check whether or not carousel is running, ensures that carousel turns off at the correct time
    double lastSeenRing = 0.0;
    final double ringDetectionTimer = 15.0;
    boolean currentlyViewingRing = false;
    enum RingState {
        FRONT, IN_MIDDLE, BACK, NONE
    }
    RingState ringState = RingState.NONE;
    double ringCount = 0;
    final double INDEX_TIME_THIRD_RING = 3.0;

    // autonomously index rings as they're pulled in by the intake
    public void index() {
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
                    } else if (ringCount == 3) {
                        if(time.milliseconds() >= INDEX_TIME_THIRD_RING) {
                            carousel.set(CAROUSEL_INDEXING_POWER);
                        } else {
                            carousel.set(0);
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
    public void resetIndex() {
        lastSeenRing = -1.0;
        carousel.set(0.0 );
        ringState = RingState.NONE;
    }



    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    boolean left_bumper;

    boolean prevRB = false;
    boolean right_bumper;
    // keep track of whether or not the shooter is currently running
    boolean isShooterRunning = false;
    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    boolean manualCarouselIndex = false;
    boolean gamepad1A;
    boolean prevG1A = false;
    final double SINGLE_INDEX_CAROUSEL_TIME = 31;
    boolean buttonIndexingActive = false;

    double prevG2Joystick = 0;
    double g2Joystick;

    // TODO: reset index variables after shooting
    void shooterRewrite() {
        right_bumper = gamepad1.right_bumper;
        left_bumper = gamepad1.left_bumper;
        gamepad1A = gamepad1.a;
        g2Joystick = gamepad2.left_stick_x;
        telemetry.addData("> Prev: ", prevLB);
        telemetry.addData("> Bumper: ", left_bumper);
        telemetry.addData("> Is shooter running", isShooterRunning);


        // check if the driver's left bumper is pressed and if it's previous value is false (prevLB)
        // if both conditions are met, toggle boolean isShooterRunning, causing the shooter to either
        // turn on or off and toggle boolean isCarouselManuallyStopped, ensuring the driver can manually
        // run the carousel
        if((left_bumper && !prevLB) || (right_bumper && !prevRB)) {

            // toggle boolean isShooterRunning, causing the shooter to either turn on or off and toggle
            // boolean isCarouselManuallyStopped, ensuring the driver can manually run the carousel
            if (!isShooterRunning) {
                if (left_bumper) {
                    manualCarouselIndex = false;
                    carouselActivationWaitTime = time.milliseconds() + CAROUSEL_ACTIVATION_TIME;
                } else if (right_bumper) {
                    manualCarouselIndex = true;
                }

                isShooterRunning = true;
            } else {
                resetIndex();
                isShooterRunning = false;
            }


        }



        // TODO: add manual index

        if (isShooterRunning) {
            shooter.set(SHOOTER_POWER);


            // if the wait time has been completed (time is greater than the wait time) and the wait time
            // has already been set, run carousel
            if (!manualCarouselIndex && time.milliseconds() > carouselActivationWaitTime) {
                // run carousel
                carousel.set(CAROUSEL_SHOOTING_POWER);
                lastSeenRing = -1;
                ringCount = 0;
            } else if (manualCarouselIndex) {
                if (gamepad1A && !prevG1A) {

                    if (buttonIndexingActive) {
                        carouselActivationWaitTime += SINGLE_INDEX_CAROUSEL_TIME;
                        ringCount--;
                    } else {
                        carouselActivationWaitTime = time.milliseconds() + SINGLE_INDEX_CAROUSEL_TIME;
                        buttonIndexingActive = true;
                    }
                    carousel.set(CAROUSEL_SHOOTING_POWER);
                }

                if (carouselActivationWaitTime < time.milliseconds()) {
                    buttonIndexingActive = false;
                    carousel.set(0);
                }
            }
        } else {
            shooter.set(0);
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
    int i = 0;

    boolean wasTouching = false;

    double wobbleRotatePower;
    double g2RightStickY;

    // bind autoWobble to operator's a, bind teleopWobble to operator's y, and bind wobbleRotate to
    // operator's right stick y (also uses the touch sensor to stop wobbleRotate)
    public void wobbleGoals() {


        // if operator's a button is pressed and it's previous value is false (prevA), check if
        // teleopServo is open (isTeleopServo). if it is, close it and if it isn't, open it. No
        // matter what, toggle isTeleopServo
        if (gamepad2.a && !prevA) {
            // if
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


        g2RightStickY = -gamepad2.right_stick_y;
        // set wobble rotate to wobbleRotatePower (declared above, is equal to operator's right stick
        // y as long as it's above the threshold)
        // if the touch sensor is pressed and the wobbleRotatePower isn't positive (meaning if the
        // operator isn't bringing the goal up), stop wobbleRotate
        if(touchSensor.isPressed()) {
            if (g2RightStickY < THRESHOLD) {
                wobbleRotatePower = 0.2;
            }
            wasTouching = true;
            i++;
        } else if (Math.abs(g2RightStickY) > THRESHOLD) {
            if (wasTouching) {
                if (g2RightStickY > THRESHOLD) {
                    wobbleRotatePower = g2RightStickY * 0.5;
                    wasTouching = false;
                }
            } else {
                wobbleRotatePower = g2RightStickY * 0.5;
            }
        }

        wobbleRotate.set(wobbleRotatePower);


        telemetry.addData("right stick y: ", g2RightStickY);
        telemetry.addData("raw right stick y: ", gamepad2.right_stick_y);
        telemetry.addData("raw right stick x: ", gamepad2.right_stick_x);
        telemetry.addData("Wobble Rotate Power", wobbleRotatePower);
        // TODO: add var if it hasnt moved up since it auto did
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
