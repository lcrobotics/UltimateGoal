package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.RevIMU;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class SuperOp extends OpMode {
    // power constants
    final float INTAKE_POWER = 1f;
    final float INTAKE_POWER_SLOW = .6f;
    final float SHOOTER_POWER = 1f;

    final double THRESHOLD = 0.12;

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
    Motor frontRightDrive;
    Motor backLeftDrive;
    Motor backRightDrive;

    // declare drive
    public MecanumDrive drive;

    // declare booleans for toggles
    boolean shooterOn = false;
    boolean frontOn = false;
    boolean topOn = false;

    @Override
    public void init() {
        // initialize non-drive motors
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);

        // set shooter to float so that we don't murder another motor
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize servos
        frontHook = new SimpleServo(hardwareMap, "FrontHook");
        topHook = new SimpleServo(hardwareMap, "TopHook");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");

        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        frontLeftDrive.setInverted(true);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, 1.2);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, 1.2);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        backRightDrive.setInverted(true);

        // initialize imu
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        // initialize drive
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    // bind rotate to operator's right stick
    // create toggle for front servo to operator's x
    // create toggle for top servo to operator's a
    public void wobbleGoals() {
        // bind rotate power to right stick of operator
        rotate.set(gamepad2.right_stick_y * .4);

        // slow down intake for wobble goal (bind to dpad left)
        if (gamepad2.left_bumper) {
            intake.set(-INTAKE_POWER_SLOW);
        } else {
            intake.set(0);
        }

        // toggles front servo on operator's x press
        if (gamepad2.x) {
            if (!frontOn) {
                frontHook.setPosition(1);
                frontOn = true;
            } else {
                frontHook.setPosition(0);
                frontOn = false;
            }
        }

        // toggles top servo on operator's a press
        if (gamepad2.a) {
            if (!topOn) {
                topHook.setPosition(1);
                topOn = true;
            } else {
                topHook.setPosition(0);
                topOn = false;
            }
        }
    }

    // toggle shooter on driver's left bumper
    public void shooter() {
        // make left bumper toggle for shooter
        if (gamepad1.left_bumper) {
            if (!shooterOn) {
                shooter.set(-SHOOTER_POWER);
                shooterOn = true;
            } else {
                shooter.set(0);
                shooterOn = false;
            }
        }
    }

    // binds intake power to driver's left trigger
    // triggers shooter servo when driver presses right bumper
    public void intake() {
        float intakePower = 0;
        // set intake as a button on right trigger and reverse intake on left trigger (button)
        if(gamepad2.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

        // set intake as a button on right trigger and reverse intake on left trigger (button)
        // and override operator
        if(gamepad1.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

        intake.set(intakePower);

        // triggers shooterServo when right bumper pressed
        if (gamepad1.right_bumper) {
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
            shooter.set(0);

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

            // release all servos
            shooterServo.setPosition(0);
            topHook.setPosition(0);
            frontHook.setPosition(0);
        }
    }

    // drive according to controller inputs from driver's sticks
    public void drive() {
        drive.driveRobotCentric(gamepad1.left_stick_y * .8, -gamepad1.left_stick_x * .8, -gamepad1.right_stick_x * .8, true);
    }
}