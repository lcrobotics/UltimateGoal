package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Set;

@TeleOp
public class Teleop extends OpMode {
    DcMotor intake;
    DcMotor linearActuator;
    DcMotor shooter;
    Servo openGrabber;
    Servo push;

    boolean grabservo = true;
    boolean pushservo = true;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        push = hardwareMap.get(Servo.class, "push");
        //Identifies
    }

    @Override
    public void loop() {
        grabbing();
        shooter();
        push();
        intake();
    }


    public void grabbing() {
        if (gamepad1.right_bumper) {
            linearActuator.setPower(1);
        }
        if (gamepad1.b) {
            grabservo = true;
        }
    }

    public void shooter() {
        if (gamepad1.left_stick_button) {
            shooter.setPower(1);
        }
    }

    public void push() {
        if (gamepad1.a) {
            pushservo = true;
        }
    }

    public void intake() {
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        }
    }
}