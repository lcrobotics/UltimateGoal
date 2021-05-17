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
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare motors
    Motor spin;
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
        spin = new Motor(hardwareMap, "spin", cpr, rpm);
        shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
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
    }

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;
    boolean LBPress = false;
    // run shoot on driver's left bumper press and run spin a second later
    public void newShooter() {
        // track history of left bumper
        if ((isLB = gamepad1.left_bumper) && !wasLB) {
            // if the shooter is on and left bumper is pressed, turn off spin and shoot, and set
            // LBPress back to false
            // if the shooter is off and left bumper is pressed, turn on shoot, then wait a second
            // and turn on spin
            if (!shooterOn) {
                // turn off spin and shoot and set LBPress to false
                spin.set(0);
                shoot.set(0);
                LBPress = false;
            } else {
                // if LBPress is false, reset time and set LBPress to true (makes sure that time
                // only gets reset once per time shooting)
                if (!LBPress) {
                    // reset time and set LBPress to true
                    time.reset();
                    LBPress = true;
                }

                // while LBPress is true wait 1000 milliseconds and then turn spin on, all while
                // keeping shoot running. after 3000 milliseconds, set LBPress to false and break out
                // of while loop
                while(LBPress) {
                    // if time is less than 1000 milliseconds, keep spin off. once time is greater
                    // than 1000 milliseconds, turn spin on
                    if (time.milliseconds() < 1000) {
                        // keep spin off
                        spin.set(0);
                    } else {
                        // turn spin on
                        spin.set(.9);
                    }
                    // keep shoot on, regardless of spin's power level
                    shoot.set(1);

                    // once time is greater than 3000 milliseconds, set LBPress to false and stop
                    // while loop
                    if(time.milliseconds() >= 3000) {
                        LBPress = false;
                    }
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
            spin.set(.9);
        } else {
            spin.set(0);
        }
    }

    // declare experimental rbg thresholds for the ring (need to be found)
    double redThreshold = .02;
    double greenThreshold = .02;
    double blueThreshold = .02;
    double alphaThreshold;
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
                spin.set(1);
                spinOn = true;
            } else if ((colors.red < redThreshold || colors.green < greenThreshold
                    || colors.blue < blueThreshold) && spinOn && notRing == 1) {
                spin.set(0);
                spinOn = false;
                notRing = 0;
                ringCount++;

            }  else if ((colors.red < redThreshold || colors.green < greenThreshold
                    || colors.blue < blueThreshold) && spinOn && notRing == 0) {
                notRing++;
                spin.set(1);
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
