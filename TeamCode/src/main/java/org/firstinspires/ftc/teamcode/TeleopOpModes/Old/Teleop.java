package org.firstinspires.ftc.teamcode.TeleopOpModes.Old;

//@TeleOp(name = "Competition TeleOp")
public class Teleop extends SuperOpOld {
    @Override
    public void loop() {
        drive();
        intake();
        shooter();
        wobbleGoals();
        driverStop();
    }
}