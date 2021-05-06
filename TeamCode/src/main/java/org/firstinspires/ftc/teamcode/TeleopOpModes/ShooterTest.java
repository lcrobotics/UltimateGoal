package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
// extends OpMode instead of SuperOp because the prototype doesn't have any of the other motors
public class ShooterTest extends OpMode {
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare motors
    Motor spin;
    Motor shoot;

    // declare elapsed time constructor
    ElapsedTime time;

    // initialize hardware and constructors
    public void init() {
        // initialize motors
        spin = new Motor(hardwareMap, "spin", cpr, rpm);
        shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
        // set shoot to reverse (to correct hardware issue)
        shoot.setInverted(true);
        // set zero power to float instead of brake so the motors don't burn out trying to stop
        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize time constructor
        time = new ElapsedTime();
    }

   // run TeleOp methods (in this case, just newShooter())
   public void loop() {
        // call TeleOp methods
        newShooter();
   }

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;
    // run both motors on driver's left bumper press
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
                // the spin motor and intake turn on after a second of the shooter running
                // this allows shoot to get up to full speed
                if (time.milliseconds() >= 1000) {
                    spin.set(1);
                }
            }
            shooterOn = !shooterOn;
        }
        wasLB = isLB;
    }
}