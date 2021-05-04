package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
// extends OpMode instead of SuperOp because the prototype doesn't have any of the other motors
public class ShooterTest extends OpMode {
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;

    // declare motors
    Motor spin;
    Motor shoot;

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;

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
    }

   // run TeleOp methods (in this case, just newShooter())
   public void loop() {
        // call TeleOp methods
        newShooter();
   }

    // run both motors on driver's left bumper press
    public void newShooter() {
        // track history of button
        if ((isLB = gamepad1.left_bumper) && !wasLB) {
            if (shooterOn) {
                // if the shooter is on and left bumper is pressed, turn shoot and spin off
                shoot.set(0);
                spin.set(0);
            } else {
                // if the shooter is off and left bumper is pressed, turn shoot and spin off
                shoot.set(1);
                spin.set(1);
            }
            shooterOn = !shooterOn;
        }
        wasLB = isLB;
    }
}