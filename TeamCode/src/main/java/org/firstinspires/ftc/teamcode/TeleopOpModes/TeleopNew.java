package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopNew extends OpMode {
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
    final double CAROUSEL_INDEXING_POWER = .5;
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;
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
    Motor shoot;
    Motor intake;

    // declare new ElapsedTime (needed for shooter)
    ElapsedTime time;
    // declare new ColorSensor (needed to index rings)
    ColorSensor colorSensor;
    // declare drive constructor
    public MecanumDrive drive;

    public void init() {
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm, 0.9);
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // multipliers on frontRightDrive and backLeftDrive are because of the weight imbalance on our robot
        // was .6
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, 0.9);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setInverted(true);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, 1);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm, 1);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setInverted(true);
        // initialize non-drive motors
        carousel = new Motor(hardwareMap, "carousel", cpr, rpm);
        shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize time constructor
        time = new ElapsedTime();
        // initialize ColorSensor
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        // initialize drive (so we can drive)
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        drive.setMaxSpeed(1);
    }

    @Override
    public void loop() {
        // call TeleOp methods
        index();
        newShooter();
        newIntake();
        drive();

        // add index telemetry
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("carousel", carousel.get());
        telemetry.addData("ringSightings", ringSightings);
        telemetry.addData("ring count", ringCount);
        telemetry.addData("time", time.milliseconds());
        telemetry.addData("wait time", carouselActivationWaitTime);
        telemetry.addData("manual carousel", isCarouselManuallyStopped);
        telemetry.addData("index wait", indexWaitSet);
        // add drive telemetry
        telemetry.addData("Front Left Power", frontLeftDrive.get());
        telemetry.addData("Front Right Power", frontRightDrive.get());
        telemetry.addData("Back Left Power", backLeftDrive.get());
        telemetry.addData("Back Right Power", backRightDrive.get());
        telemetry.addData("Shooter Power", shoot.motor.getPower());
    }


    // drive according to controller inputs from driver's sticks
    public void drive() {
        double strafePower = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
        double forwardPower = Math.abs(gamepad1.left_stick_y) < 0.1 ? 0 : gamepad1.left_stick_y;
        double turnPower = Math.abs(gamepad1.right_stick_x) < 0.1 ? 0 : gamepad1.right_stick_x;

        drive.driveRobotCentric(
                -strafePower * 0.8,
                -forwardPower * 0.8,
                turnPower * 0.7,
                true
        );
    }

    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    // shooter booleans (for toggle)
    boolean isShooterRunning = false;
    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    // ensure that the carousel's wait time is only set once per toggle
    boolean isCarouselWaitSet = false;
    // ensure that if the carousel is run manually, the carousel will not run again on the same toggle
    boolean isCarouselManuallyStopped = false;
    // run shoot on driver's left bumper press and run spin a second later
    public void newShooter() {
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
            shoot.set(0);
            isCarouselWaitSet = false;
        }

        // if isShooterRunning is true, turn on shooter
        if(isShooterRunning) {
            // turn on shooter
            shoot.set(SHOOTER_POWER);
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
    double alphaThreshold = 250;
    // keeps track of how many times the color sensor hasn't seen a ring
    int ringSightings = 0;
    // keep track of rings indexed
    int ringCount = 0;

    boolean indexWaitSet = false;
    double carouselIndexWaitTime = 0.0;
    final double FALSE_POSITIVE_BUFFER = 50.0;
    // autonomously index rings as they are pulled in by the intake
    public void index() {
        /*if(ringCount < 2) {
            if(!indexWaitSet) {
                carouselIndexWaitTime = time.milliseconds() + FALSE_POSITIVE_BUFFER;
                indexWaitSet = true;
            }

            if(alphaThreshold < colorSensor.alpha() && carouselIndexWaitTime < time.milliseconds()) {
                ringSightings++;
                carousel.set(CAROUSEL_INDEXING_POWER);
                indexWaitSet = false;
            }

            if(!indexWaitSet) {
                carouselIndexWaitTime = time.milliseconds() + FALSE_POSITIVE_BUFFER;
                indexWaitSet = true;
            }

            if (alphaThreshold > colorSensor.alpha() && ringSightings == 1
                    && carouselIndexWaitTime < time.milliseconds()) {
                carousel.set(CAROUSEL_INDEXING_POWER);
                indexWaitSet = false;
            }

            if(!indexWaitSet) {
                carouselIndexWaitTime = time.milliseconds() + FALSE_POSITIVE_BUFFER;
                indexWaitSet = true;
            }

            if(alphaThreshold < colorSensor.alpha() && ringSightings == 1
                    && carouselIndexWaitTime < time.milliseconds()) {
                ringSightings++;
                carousel.set(CAROUSEL_INDEXING_POWER);
                indexWaitSet = false;
            }

            if(!indexWaitSet) {
                carouselIndexWaitTime = time.milliseconds() + FALSE_POSITIVE_BUFFER;
                indexWaitSet = true;
            }

            if (alphaThreshold > colorSensor.alpha() && ringSightings == 2
                    && carouselIndexWaitTime < time.milliseconds()) {
                carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                carousel.set(0);
                ringCount++;
                ringSightings = 0;
                indexWaitSet = false;
            }
        } */



        if(gamepad1.right_bumper) {
            carousel.set(CAROUSEL_INDEXING_POWER);
        } else {
            carousel.set(0);
        }
    }
}
