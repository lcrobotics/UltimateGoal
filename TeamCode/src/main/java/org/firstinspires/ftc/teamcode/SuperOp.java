package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SuperOp extends OpMode {
    //motors/servos for exactly what it says
    DcMotor Intake;
    DcMotor Rotator;
    DcMotor Shooter;
    Servo FrontHook;
    Servo TopHook;
    Servo ShooterServo;
    //wheels
    DcMotor FrontLeftDrive;
    DcMotor BackLeftDrive;
    DcMotor FrontRightDrive;
    DcMotor BackRightDrive;

    @Override
    public void init() {
        // Initializes each piece of hardware
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Rotator = hardwareMap.get(DcMotor.class, "Rotator");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        FrontHook = hardwareMap.get(Servo.class, "FrontHook");
        TopHook = hardwareMap.get(Servo.class, "TopHook");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");

        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
    }
}
