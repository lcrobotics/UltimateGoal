package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ShooterIntakeSensor extends OpMode {
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare motors
    Motor spin;
    Motor shoot;
    Motor intake;

    // declare new ElapsedTime (needed for shooter)
    ElapsedTime time;
    // declare new color sensor
    NormalizedColorSensor colorSensor;

    public void init() {
        // initialize motors
        spin = new Motor(hardwareMap, "spin", cpr, rpm);
        shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        // set shoot to reverse (to correct hardware issue)
        shoot.setInverted(true);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize time constructor
        time = new ElapsedTime();

        // initialize color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    }

    @Override
    public void loop() {
        // call TeleOp methods
        newShooter();
        newIntake();

        if (!gamepad1.left_bumper) {
            spin();
        }
    }

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;
    // run shoot on driver's left bumper press and run spin and intake a second later
    public void newShooter() {
        // track history of button
        if ((isLB = gamepad1.left_bumper) && !wasLB) {
            if (shooterOn) {
                // if the shooter is on and left bumper is pressed, turn shoot and spin off
                shoot.set(0);
                spin.set(0);
            } else {
                // if the shooter is off and left bumper is pressed, turn shoot on
                shoot.set(1);
                // give the shoot motor a second to spin up before putting any rings in
                // the spin motor turn on after a second of the shooter running
                // this allows shoot to get up to full speed
                if (time.milliseconds() >= 1000) {
                    spin.set(1);
                }
            }
            shooterOn = !shooterOn;
        }
        wasLB = isLB;
    }

    // takes care of motors pertaining to the intake (intake, reverse intake, and spin)
    public void newIntake () {
        // if driver presses right trigger, turn on intake
        // if driver presses left trigger, reverse intake (useful if rings get stuck somehow)
        if(gamepad1.right_trigger > .12) {
            intake.set(1);
        } else if (gamepad1.left_trigger > .12) {
            intake.set(-1);
        } else {
            intake.set(0);
        }

        // if driver presses right bumper, turn on spin (allowing rings to be pulled up before shooting)
        if (gamepad1.right_bumper) {
            spin.set(1);
        } else {
            spin.set(0);
        }
    }

    // Get the normalized colors from the sensor
    NormalizedRGBA colors = colorSensor.getNormalizedColors();
    // declare experimental rbg thresholds for the ring
    double redThreshold;
    double greenThreshold;
    double blueThreshold;

    // used to make sure spin turns off at the right time
    boolean spinOn = false;
    int notRing = 0;
    int ringCount = 0;
    public void spin() {
        // shooter should override, counter of ring indices on last ring does not run spin
        // if first time seeing ring turn on spin
        // if second time seeing ring wait until can't see it again
        if (ringCount != 2) {
            if(colors.red >= redThreshold && colors.green >= greenThreshold
                    && colors.blue >= blueThreshold && !spinOn) {
                spin.set(.4);
                spinOn = true;
            } else if (colors.red < redThreshold && colors.green < greenThreshold
                    && colors.blue < blueThreshold && spinOn && notRing == 1) {
                spin.set(0);
                spinOn = false;
                notRing = 0;
                ringCount++;

            }  else if (colors.red < redThreshold && colors.green < greenThreshold
                    && colors.blue < blueThreshold && spinOn && notRing == 0) {
                notRing++;
            }
        }
    }

    public void getColor() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // print rbg values as telemetry
        telemetry.addLine()
                .addData("Red", colors.red)
                .addData("Green",  colors.green)
                .addData("Blue", colors.blue);
        telemetry.update();
    }
}
