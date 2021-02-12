package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpEx extends OpMode {
    // declare motors for each part of robot
    // for intake conveyor
    DcMotor intake;
    // for wobble goal grabber - pushes mechanism over wall
    DcMotor linearActuator;
    // for shooter - launch rings out of intake
    DcMotor shooter;

    // declare servos for each part of the robot
    // for wobble goal grabber - opens latch to grab goal
    Servo grabber;
    // for pushing rings out of the intake and into the shooter
    Servo push;

    // declare booleans for servos (use to power servos)
    // use for grabber
    boolean grabServo = true;
    // use for push
    boolean pushServo = true;

    @Override
    public void init() {
        // initialize motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // initialize servos
        grabber = hardwareMap.get(Servo.class, "grabber");
        push = hardwareMap.get(Servo.class, "push");
    }

    @Override
    public void loop() {
        // call all methods
        grabber();
        shooter();
        push();
        intake();
    }

    // power linear actuator on right bumper press (to push mechanism over wall)
    // and power grabber onn b press (to grab wobble goal)
    public void grabber() {
        // set motor power to 1 when right bumper is pressed
        if (gamepad1.right_bumper) {
            linearActuator.setPower(1);
        }
        // starts servo when b is pressed
        if (gamepad1.b) {
            grabServo = true;
        }
    }

    // power shooter motor on left stick button press (used to launch rings at power shots/goals)
    public void shooter() {
        // sets power to 1 when left stick button is pressed
        if (gamepad1.left_stick_button) {
            shooter.setPower(1);
        }
    }

    // power push servo when on a press (to push rings from intake into shooter)
    public void push() {
        // starts servo when a is pressed
        if (gamepad1.a) {
            pushServo = true;
        }
    }

    // power intake motor on left bumper press (to get rings into robot)
    public void intake() {
        // sets power to 1 when left bumper is pressed
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        }
    }
}
