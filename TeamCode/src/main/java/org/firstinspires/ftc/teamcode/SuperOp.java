package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.CommandCenter.hardware.RevIMU;
import com.lcrobotics.easyftclib.CommandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.CommandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SuperOp extends OpMode {
    // power constant
    final double ROTATE_POWER = .6;
    final double INTAKE_POWER = .5;

    // drive constants
    final int cpr = 448;
    final int rpm = 64;

    // declare imu
    RevIMU imu;

    // declare non-drive motors
    Motor intake;
    Motor rotate;
    Motor shooter;

    // declare servos
    ServoEx frontHook;
    ServoEx topHook;
    ServoEx shooterServo;

    // declare drive motors
    Motor frontLeftDrive;
    Motor backLeftDrive;
    Motor frontRightDrive;
    Motor backRightDrive;

    // declare drive
    public MecanumDrive drive;

    @Override
    public void init() {
        // initialize non-drive motors
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);

        // initialize servos
        frontHook = new SimpleServo(hardwareMap, "FrontHook");
        topHook = new SimpleServo(hardwareMap, "TopHook");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");

        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        frontLeftDrive.setInverted(true);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        backRightDrive.setInverted(true);

        // initialize imu
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        // initialize drive
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    // bind rotate up to operator's left stick button and down to operator's right stick button
    // bind operator's x to trigger and hold front servo
    // bind operator's y to release front servo
    // bind operator's a to trigger and hold top servo
    // bind operator's b to release top servo
    public void wobbleGoals() {
        // set rotate to -rotate power on left stick button press
        if (gamepad2.left_stick_button) {
            rotate.set(-ROTATE_POWER);
        }
        // set rotate to rotate power on right stick button press
        if (gamepad2.right_stick_button) {
            rotate.set(ROTATE_POWER);
        }

        if (gamepad2.dpad_up) {
            intake.set(-INTAKE_POWER);
        } else {
            intake.set(0);
        }
        // triggers and holds front servo on x press
        if (gamepad2.x) {
            frontHook.setPosition(1);
        }
        // releases front servo on y press
        if (gamepad2.y) {
            frontHook.setPosition(0);
        }
        // triggers and holds top servo on a press
        if (gamepad2.a) {
            topHook.setPosition(1);
        }
        // releases top servo on b press
        if (gamepad2.b){
            topHook.setPosition(0);
        }
    }

    // binds shooter to right trigger for operator
    public void shooter() {
        // binds shooter power to operator's right trigger
        shooter.set(-gamepad2.right_trigger);
    }

    // binds intake power to left stick y for operator
    // binds shooterServo to right bumper and sets power for operator
    public void intake() {
        // bind intake to power of left stick y for operator
        intake.set(gamepad2.left_stick_y);
        // triggers shooterServo when right bumper pressed
        if (gamepad2.right_bumper) {
            shooterServo.setPosition(1);
        } else {
            shooterServo.setPosition(0);
        }
    }

    // binds stoppage of all motors/servos to dpad down (both operator and driver)b
    public void stop() {
        // if driver presses dpad down, stop all motors/servos
        if (gamepad1.dpad_down) {
            // stop non-drive motors
            intake.set(0);
            rotate.set(0);
            shooter.set(0);

            // stop servos
            frontHook.setPosition(0);
            topHook.setPosition(1);
            shooterServo.setPosition(0);

            // stop drive motors
            frontLeftDrive.set(0);
            frontRightDrive.set(0);
            backLeftDrive.set(0);
            backRightDrive.set(0);
        }
        // if operator presses dpad down, stop all motors/servos
        if (gamepad2.dpad_down) {
            // stop non-drive motors
            intake.set(0);
            rotate.set(0);
            shooter.set(0);

            // stop servos
            frontHook.setPosition(0);
            topHook.setPosition(1);
            shooterServo.setPosition(0);

            // stop drive motors
            frontLeftDrive.set(0);
            frontRightDrive.set(0);
            backLeftDrive.set(0);
            backRightDrive.set(0);
        }
    }

    // drive according to controller inputs from driver's sticks
    public void drive() {
        drive.driveRobotCentric(gamepad1.left_stick_x * .8, -gamepad1.left_stick_y * .8, gamepad1.right_stick_x * .8, true);
    }
}