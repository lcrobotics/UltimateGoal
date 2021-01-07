package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


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

    final double INTAKE_POWER = 1;
    final double GRABBER_POWER = 1;
    final double SHOOTER_POWER = 1;


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

    // this method binds the grabber function to right bumper
    public void grabbing() {
        // set motor power to 1 when right bumper is pressed
        if (gamepad1.right_bumper) {
            linearActuator.setPower(GRABBER_POWER);
        }
        // starts servo when b is pressed   
        if (gamepad1.b) {
            grabservo = true;
        }
    }
    // binds shooter to left stick button .
    public void shooter() {
        // sets power to 1 when left stick button s pressed
        if (gamepad1.left_stick_button) {
            shooter.setPower(SHOOTER_POWER);
        }
    }
    // binds pushservo to a and sets power
    public void push() {
        // starts servo when a is pushed
        if (gamepad1.a) {
            pushservo = true;
        }
    }

    // binds intake to left bumper and sets power
    public void intake() {
        // sets power to 1 when left bumper is pressed
        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_POWER);
        }
    }
    // reverse intake binded to right bumper and sets power to opposite of intake
    public void reverseIntake() {
        if (gamepad1.right_bumper) {
            intake.setPower(-INTAKE_POWER);
        }
    }
    // binds stoppage of all motors/servos to dpad down
    public void stop() {
        //stops all functions
        if (gamepad1.dpad_down) {
            linearActuator.setPower(0);
            grabservo = false;
            shooter.setPower(0);
            pushservo = false;
            intake.setPower(0);
        }
    }
}