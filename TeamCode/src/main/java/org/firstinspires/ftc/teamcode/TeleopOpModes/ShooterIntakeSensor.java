package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
    NormalizedColorSensor colorSensorNormalized;
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
        colorSensorNormalized = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        // initialize ColorSensor
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
    }

    @Override
    public void loop() {
        // call TeleOp methods
        newShooter();
        newIntake();
        index();

        telemetry.addData("time", time.milliseconds());
        telemetry.addData("wait", carouselActivationWaitTime);
        telemetry.addData("carousel", carousel.get());
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

    // declare experimental rbg thresholds for the ring (need to be found)
    double redThreshold = .02;
    double greenThreshold = .02;
    double blueThreshold = .02;
    double alphaThreshold = 450;
    // used to make sure spin turns off at the right time
    boolean spinOn = false;
    int notRing = 0;
    int ringCount = 0;
    public void index() {
        NormalizedRGBA colors = colorSensorNormalized.getNormalizedColors();

        /*telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha()); */
        /*telemetry.addLine()
                .addData("RedNormalized", colors.red)
                .addData("GreenNormalized",  colors.green)
                .addData("BlueNormalized", colors.blue);
        telemetry.addData("red threshold", redThreshold);
        telemetry.addData("green threshold", greenThreshold);
        telemetry.addData("blue threshold", blueThreshold);
        telemetry.addData("notRing", notRing);
        telemetry.addData("ring count", ringCount);
        telemetry.addData("spin on?", spinOn);
        telemetry.update(); */

        if (ringCount != 2) {
            if((colors.red >= redThreshold || colors.green >= greenThreshold
                    || colors.blue >= blueThreshold) && spinOn) {
                carousel.set(1);
                spinOn = true;
            } else if ((colors.red < redThreshold || colors.green < greenThreshold
                    || colors.blue < blueThreshold) && spinOn && notRing == 1) {
                carousel.set(0);
                spinOn = false;
                notRing = 0;
                ringCount++;

            }  else if ((colors.red < redThreshold || colors.green < greenThreshold
                    || colors.blue < blueThreshold) && spinOn && notRing == 0) {
                notRing++;
                carousel.set(1);
            }
        }
        // shooter should override, counter of ring indices on last ring does not run spin
        // if first time seeing ring turn on spin
        // if second time seeing ring wait until can't see it again
        /*if (ringCount < 3) {
            if (colorSensor.alpha > alphaThreshold && !spinOn) {
                spin.set(1);
                spinOn = true;
            } else if (colorSensor.alpha < alphaThreshold && notRing == 0) {
                notRing = 1;
                spin.set(1);
            } else if (colorSensor.alpha > alphaThreshold && notRing == 1) {
                spin.set(1);
            } else if (colorSensor.alpha < alphaThreshold && notRing == 1) {
                spin.set(0);
                notRing = 0;
                ringCount++;
                spinOn = false;
            } else {
                spin.set(0);
            }
        } else if (ringCount == 3) {
            if (colors.red > redThreshold && !spinOn) {
                spin.set(1);
                spinOn = true;
            } else if (colors.red < redThreshold && notRing == 0) {
                notRing = 1;
                spin.set(1);
            } else if (colors.red > redThreshold && notRing == 1) {
                spin.set(0);
                notRing = 0;
                spinOn = false;
                ringCount++;
            } else {
                spin.set(0);
            }
        } */
    }
}
