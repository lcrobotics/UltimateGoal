package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Set;

@TeleOp
public class Teleop extends OpMode {
    // Defines each piece of hardware
    DcMotor intake;
    DcMotor linearActuator;
    DcMotor shooter;
    Servo openGrabber;
    Servo push;

    boolean grabservo = true;
    boolean pushservo = true;

    @Override
    public void init() {
        // Initializes each piece of hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        openGrabber = hardwareMap.get(Servo.class, "openGrabber");
        push = hardwareMap.get(Servo.class, "push");
    }

    @Override
    public void loop() {
        // calls all methods
        grabbing();
        shooter();
        push();
        intake();
    }

    // Code for the grabber. Servo grabs the ring and the linear actuator lifts it up.
    public void grabbing() {
        // set motor power to 1 when right bumper is pressed
        if (gamepad1.right_bumper) {
            linearActuator.setPower(1);
        }
        // starts servo when b is pressed
        if (gamepad1.b) {
            grabservo = true;
        }
    }
    // code for the ring shooter which shoots the rings.
    public void shooter() {
        // sets power to 1 when left stick button s pressed
        if (gamepad1.left_stick_button) {
            shooter.setPower(1);
        }
    }
    // code for the push servo that pushes the rings into the shooter.
    public void push() {
        // starts servo when a is pressed
        if (gamepad1.a) {
            pushservo = true;
        }
    }

    // code for the intake motor which brings the rings into the robot.
    public void intake() {
        // sets power to 1 when left bumper is pressed
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        }
    }
}