package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SuperOp extends OpMode {
    final int cpr = 448;
    final int rpm = 64;
    //motors/servos for exactly what it says
    Motor Intake;
    Motor Rotator;
    Motor Shooter;
    Servo FrontHook;
    Servo TopHook;
    Servo ShooterServo;
    //wheels
    Motor FrontLeftDrive;
    Motor BackLeftDrive;
    Motor FrontRightDrive;
    Motor BackRightDrive;

    @Override
    public void init() {
        // Initializes each piece of hardware
        Intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        Rotator = new Motor(hardwareMap, "Rotator", cpr, rpm);
        Shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);
        FrontHook = hardwareMap.get(Servo.class, "FrontHook");
        TopHook = hardwareMap.get(Servo.class, "TopHook");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");

        FrontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        BackLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        FrontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        BackRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
    }
}
