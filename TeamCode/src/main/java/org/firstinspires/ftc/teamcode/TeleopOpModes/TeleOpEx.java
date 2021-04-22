package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpEx extends OpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    public double x_speed;
    public double y_speed;
    public double w_speed;

    public double dead_zone;

    @Override
    public void init() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive  = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        x_speed = .8;
        y_speed = .8;
        w_speed = .8;

        dead_zone = .05;
    }

    @Override
    public void loop() {
        drive();

        telemetry.addData("Front Left Power", frontLeftDrive.getPower());
        telemetry.addData("Front Right Power", frontRightDrive.getPower());
        telemetry.addData("Back Left Power", backLeftDrive.getPower());
        telemetry.addData("Back Right Power", backRightDrive.getPower());
    }

    public void drive() {
        double strafePower = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
        double forwardPower = Math.abs(gamepad1.left_stick_y) < 0.1 ? 0 : gamepad1.left_stick_y;
        double turnPower = Math.abs(gamepad1.right_stick_x) < 0.1 ? 0 : gamepad1.right_stick_x;

        teleDrive(strafePower, forwardPower, -turnPower);
    }

    public void teleDrive(double x, double y, double w) {
        frontLeftDrive.setPower((y_speed * y) + (x_speed * x) + (w_speed* w));
        frontRightDrive.setPower((y_speed * y) - (x_speed * x) - (w_speed * w));
        backLeftDrive.setPower(((y_speed * y) - (x_speed * x) + (w_speed * w)));
        backRightDrive.setPower((y_speed * y) + (x_speed * x) - (w_speed * w));
    }
}
