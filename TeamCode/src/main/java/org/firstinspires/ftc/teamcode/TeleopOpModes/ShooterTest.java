package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ShooterTest extends OpMode {
    // drive constants
    final int cpr = 448;
    final int rpm = 64;

    // declare Motors
    Motor spin;
    Motor shoot;

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;

   public void init() {
       // initialize motors
       spin = new Motor(hardwareMap, "spin", cpr, rpm);
       spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
       shoot = new Motor(hardwareMap, "shoot", cpr, rpm);
       shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
   }

   public void loop() {
       newShooter();
   }

   public void newShooter() {
       // track history of button
       if ((isLB = gamepad1.left_bumper) && !wasLB) {
           if (shooterOn) {
               // if the shooter is on and left bumper is pressed, turn shoot and spin off
               shoot.set(0);
               spin.set(0);
           } else {
               shoot.set(1);
               spin.set(1);
           }
           shooterOn = !shooterOn;
       }
       wasLB = isLB;
   }
}
