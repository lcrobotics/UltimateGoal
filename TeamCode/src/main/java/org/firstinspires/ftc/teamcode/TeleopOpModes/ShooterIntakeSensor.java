package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ShooterIntakeSensor extends OpMode {
    // declare class constants
    // constant used to update the current time and get a wait time for carousel activation
    final double CAROUSEL_ACTIVATION_TIME = 1000.0;
    // constant used for the shooter power
    final double SHOOTER_POWER = 1;
    // constant used for the carousel's power
    final double CAROUSEL_POWER = .9;
    // constant used for the intake's power
    final double INTAKE_POWER = 1;
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
    // declare new NormalizedColorSensor (needed to index rings)
    //NormalizedColorSensor colorSensorNormalized;
    // declare new ColorSensor (needed to index rings)
    ColorSensor colorSensor;

    public void init() {
        // initialize motors
        carousel = new Motor(hardwareMap, "spin", cpr, rpm);
        shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize time constructor
        time = new ElapsedTime();

        // initialize NormalizedColorSensor
        //colorSensorNormalized = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        // initialize ColorSensor
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
    }

    @Override
    public void loop() {
        // call TeleOp methods
        newShooter();
        newIntake();
        index();

        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("carousel", carousel.get());
        telemetry.addData("notRing", notRing);
        telemetry.addData("ring count", ringCount);
        telemetry.addData("carousel on?", isCarouselRunning);
    }


    // will be set to value of necessary wait time in method (eg: now + 1000 milliseconds)
    double carouselActivationWaitTime = 0.0;
    // shooter booleans (for toggle)
    boolean isShooterRunning = false;
    // set equal to the previous value of driver's left bumper
    boolean prevLB = false;
    // ensure that the carousel's wait time is only set once per toggle
    boolean isCarouselWaitSet = false;
    // run shoot on driver's left bumper press and run spin a second later
    public void newShooter() {
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
        if(time.milliseconds() > carouselActivationWaitTime && isCarouselWaitSet) {
            // run carousel
            carousel.set(CAROUSEL_POWER);
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

        // if driver presses right bumper, turn on spin (allowing rings to be pulled up before shooting)
        if (gamepad1.right_bumper) {
            carousel.set(CAROUSEL_POWER);
        } else {
            carousel.set(0);
        }
    }

    // declare threshold for color sensor data
    double alphaThreshold = 300;
    // check whether or not carousel is running, ensures that carousel turns off at the correct time
    boolean isCarouselRunning = false;
    // keeps track of how many times the color sensor hasn't seen a ring
    int notRing = 0;
    // keep track of rings indexed
    int ringCount = 0;
    // autonomously index rings as they're pulled in by the intake
    public void index() {
        // check if driver's a is pressed. if it is, begin indexing
        if(gamepad1.a) {
            // if there are 0, 1, or 2 rings currently indexed, turn carousel off after the color
            // sensor doesn't see the ring a second time
            // if there are 3 rings indexed/pulled in by the intake, turn carousel off after the
            // second time the color sensor sees the ring
            if (ringCount < 3) {
                // if the ring is seen and the carousel is not yet running, turn on the carousel and
                // set isCarouselRunning to true
                // if the ring is not seen and it's the first time it hasn't seen a ring, increment
                // notRing
                // if the ring is not seen an d it's the second time it hasn't seen the ring, turn
                // off carousel, reset notRing, increment ringCount, and reset isCarouselRunning
                // if all else fails, stop carousel
                if (colorSensor.alpha() > alphaThreshold && !isCarouselRunning) {
                    // turn on carousel and set isCarouselRunning to true
                    carousel.set(.4);
                    isCarouselRunning = true;
                } else if (colorSensor.alpha() < alphaThreshold && notRing == 0 && isCarouselRunning) {
                    // increment notRing
                    carousel.set(.4);
                    notRing = 1;
                } else if (colorSensor.alpha() > alphaThreshold && isCarouselRunning) {
                    carousel.set(.4);
                } else if (colorSensor.alpha() < alphaThreshold && notRing == 1 && isCarouselRunning) {
                    // turn off carousel, reset notRing, increment ringCount, and reset isCarouselRunning
                    carousel.set(0);
                    notRing = 0;
                    ringCount++;
                    isCarouselRunning = false;
                } else {
                    // stop carousel
                    carousel.set(0);
                }
            } else if (ringCount == 3) {
                // if the ring is seen and the carousel is not yet running, turn on the carousel and
                // set isCarouselRunning to true
                // if the ring is not seen and it's the first time it hasn't seen a ring, increment
                // notRing
                // if the ring is seen for a second time, turn off carousel, reset notRing, reset
                // ringCount, and reset isCarouselRunning
                // if all else fails, stop carousel
                if (colorSensor.alpha() > alphaThreshold && !isCarouselRunning) {
                    // turn on carousel and set isCarouselRunning to true
                    carousel.set(CAROUSEL_POWER);
                    isCarouselRunning = true;
                } else if (colorSensor.alpha() < alphaThreshold && notRing == 0 && isCarouselRunning) {
                    // increment not ring
                    notRing = 1;
                } else if (colorSensor.alpha() > alphaThreshold && notRing == 1 && isCarouselRunning) {
                    // turn off carousel, reset notRing, reset ringCount, and reset isCarouselRunning
                    carousel.set(0);
                    notRing = 0;
                    isCarouselRunning = false;
                    ringCount = 0;
                } else {
                    // turn off carousel
                    carousel.set(0);
                }
            }
        }
    }
}
