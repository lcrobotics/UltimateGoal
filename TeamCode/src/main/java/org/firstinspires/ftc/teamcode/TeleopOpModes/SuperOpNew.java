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
    // constant used to update the current time and get a wait time for carousel activation during
    // index() - makes sure the sensor doesn't see any false positives
    final double FALSE_POSITIVE_BUFFER = 50.0;
    // constant used to update the current time and get the correct run time for the carousel while
    // indexing
    final double CAROUSEL_INDEX_LENGTH = 500.0;
    // constant used for the carousel's power while indexing
    final double CAROUSEL_INDEXING_POWER = .5;
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
        telemetry.addData("index wait", indexWaitSet);
        // add shooter telemetry
        telemetry.addData("Shooter Power", shooter.motor.getPower());
        telemetry.addData("manual carousel", isCarouselManuallyStopped);
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

    // allows for the power to only be set once, making the code more efficient
    double intakePower = 0;
    // binds intake to right trigger, reverse intake to left trigger
    // both driver and operator can intake, but driver has precedence
    public void intake() {
        // set intake as a button on right trigger and reverse intake on left trigger (button)
        if (gamepad2.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

        // set intake as a button on right trigger and reverse intake on left trigger (button)
        // and override operator
        if (gamepad1.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

        // set intake to the double intake power
        intake.set(intakePower);
    }

    // declare threshold for color sensor data
    double alphaThreshold = 250;
    // keep track of rings indexed
    int ringCount = 0;
    // keep track of whether or not a wait time has been set
    boolean indexWaitSet = false;
    // will be set to value of necessary wait time in method (eg: now + 50 milliseconds)
    double carouselIndexWaitTime = 0.0;
    // index rings (can be done manually on driver's right bumper or automatically after driver's x press)
    public void index() {
        // if the driver's x button is pressed and there are less than 2 rings indexed, set a wait time
        // (to ensure no false positives), then use color sensor to detect the ring. If the color
        // sensor sees the ring and the wait time is finished, then turn on carousel until it has
        // finished indexing (determined by the time)
        if(gamepad1.x && ringCount < 2) {
            // if a wait time has not been set, set carouselIndexWaitTime to now + the buffer time
            // and set the boolean to true
            if (!indexWaitSet) {
                carouselIndexWaitTime = time.milliseconds() + FALSE_POSITIVE_BUFFER;
                indexWaitSet = true;
            }

            // if the color sensor sees the ring and has waited the correct amount of time, turn on
            // the carousel as until the time is greater than the assigned indexing time. after the
            // time is up, turn off carousel, increment ringCount, and declare that the wait time has
            // not been set
            if (alphaThreshold < colorSensor.alpha() && carouselIndexWaitTime < time.milliseconds()) {
                if(time.milliseconds() < time.milliseconds() + CAROUSEL_INDEX_LENGTH) {
                    carousel.set(CAROUSEL_INDEXING_POWER);
                } else {
                    carousel.set(0);
                    ringCount++;
                    indexWaitSet = false;
                }
            }
        }

        // if driver presses right bumper, turn on carousel at half power, otherwise stop carousel
        if(gamepad1.right_bumper) {
            carousel.set(CAROUSEL_INDEXING_POWER);
        } else {
            carousel.set(0);
        }
    }
    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    // keep track of whether or not the shooter is currently running
    boolean isShooterRunning = false;
    // ensure that if the carousel is run manually, the carousel will not run again on the same toggle
    boolean isCarouselManuallyStopped = false;
    // ensure that the carousel's wait time is only set once per toggle
    boolean isCarouselWaitSet = false;
    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    // run shooter on driver's left bumper press and run spin a second later
    public void shooter() {
        // check if the driver's left bumper is pressed and if it's previous value is false (prevLB)
        // if both conditions are met, toggle boolean isShooterRunning, causing the shooter to either
        // turn on or off and toggle boolean isCarouselManuallyStopped, ensuring the driver can manually
        // run the carousel
        if(gamepad1.left_bumper && !prevLB) {
            // toggle boolean isShooterRunning, causing the shooter to either turn on or off and toggle
            // boolean isCarouselManuallyStopped, ensuring the driver can manually run the carousel
            isShooterRunning = !isShooterRunning;
            isCarouselManuallyStopped = false;
        }

        // if isShooterRunning is false, stop both motors and reset isCarouselWaitSet back to false
        if(!isShooterRunning) {
            // stop both motors and reset isCarouselWaitSet back to false
            carousel.set(0);
            shooter.set(0);
            isCarouselWaitSet = false;
        }

        // if isShooterRunning is true, turn on shooter
        if(isShooterRunning) {
            // turn on shooter
            shooter.set(SHOOTER_POWER);
        }

        // if the driver's right bumper (manual index) isn't pressed and hasn't been pressed during
        // the current toggle, run carousel after a second
        // if the right bumper is pressed or index() hasn't been ran during the current toggle, run
        // index() and set isCarouselManuallyStopped to true
        if (!gamepad1.right_bumper && !isCarouselManuallyStopped) {
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
        } else {
            // run index() and set isCarouselManuallyStopped to true
            index();
            isCarouselManuallyStopped = true;
        }

        // set prevLB to the driver's left bumper
        prevLB = gamepad1.left_bumper;
    }

    // set equal to the previous value of operator's a button
    boolean prevA = false;
    // keep track of whether or not the teleopServo is currently open
    boolean isTeleopServo = false;
    // set equal to the previous value of operator's y button
    boolean prevY = false;
    // keep track of whether or not the autoServo is currently open
    boolean isAutoServo = false;
    // bind autoWobble to operator's a, bind teleopWobble to operator's y, and bind wobbleRotate to
    // operator's right stick y (also uses the touch sensor to stop wobbleRotate)
    public void wobbleGoals() {
        // double that allows for neater method code (checks for threshold)
        double wobbleRotatePower = Math.abs(gamepad2.right_stick_y) < THRESHOLD ? 0 : gamepad2.right_stick_y;

        // if operator's a button is pressed and it's previous value is false (prevA), check if
        // teleopServo is open (isTeleopServo). if it is, close it and if it isn't, open it. No
        // matter what, toggle isTeleopServo
        if (gamepad2.a && !prevA) {
            // if
            if (isTeleopServo) {
                // if servo is open, close on a press
                teleopWobble.setPosition(.93);
            } else {
                // if servo is closed, open on a press
                teleopWobble.setPosition(.3);
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
                autoWobble.setPosition(0);
            } else {
                // if servo is closed, open on y press
                autoWobble.setPosition(0.5);
            }
            // set isAutoServo to it's opposite (toggle)
            isAutoServo = !isAutoServo;
        }
        // set prevY to the current value of operator's y
        prevY = gamepad2.y;

        // set wobble rotate to wobbleRotatePower (declared above, is equal to operator's right stick
        // y as long as it's above the threshold)
        wobbleRotate.set(wobbleRotatePower);
        // if the touch sensor is pressed and the wobbleRotatePower isn't positive (meaning if the
        // operator isn't bringing the goal up), stop wobbleRotate
        if(touchSensor.isPressed() && wobbleRotatePower < 0) {
            wobbleRotate.set(0);
        }
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
            teleopWobble.setPosition(.93);
            autoWobble.setPosition(0);
        }
    }
}
