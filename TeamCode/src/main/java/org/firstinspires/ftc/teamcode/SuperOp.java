package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SuperOp extends OpMode {
    //motors/servos for exactly what it says
    DcMotor intake;
    DcMotor linearActuator;
    DcMotor shooter;
    Servo openGrabber;
    Servo push;
    //wheels
    DcMotor FrontLeftDrive;
    DcMotor BackLeftDrive;
    DcMotor FrontRightDrive;
    DcMotor BackRightDrive;

    @Override
    public void init() {
        // Initializes each piece of hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        push = hardwareMap.get(Servo.class, "push");
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

    }
}
