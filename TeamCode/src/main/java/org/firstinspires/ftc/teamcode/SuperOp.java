package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SuperOp extends OpMode {
    // power constants
    final double INTAKE_POWER = 1;
    final double ROTATE_POWER = 1;
    final double SHOOTER_POWER = 1;

    // drive constants
    final int cpr = 448;
    final int rpm = 64;

    // servo constants
    boolean frontHookServo = true;
    boolean topHookServo = true;
    boolean shooterServo = true;

    // declare non-drive motors
    Motor Intake;
    Motor Rotate;
    Motor Shooter;

    // declare servos
    Servo FrontHook;
    Servo TopHook;
    Servo ShooterServo;

    // declare drive motors
    Motor FrontLeftDrive;
    Motor BackLeftDrive;
    Motor FrontRightDrive;
    Motor BackRightDrive;

    // declare drive
    public MecanumDrive drive;

    @Override
    public void init() {
        // initialize non-drive motors
        Intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        Rotate = new Motor(hardwareMap, "Rotator", cpr, rpm);
        Shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);

        // initialize servos
        FrontHook = hardwareMap.get(Servo.class, "FrontHook");
        TopHook = hardwareMap.get(Servo.class, "TopHook");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");

        // initialize drive motors
        FrontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        BackLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        FrontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        BackRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);

        // initialize drive
        drive = new MecanumDrive(true, FrontLeftDrive, FrontRightDrive, BackLeftDrive, BackRightDrive);
    }

    // binds Rotate to y and sets power
    // binds front servo to b and starts servo
    // binds top servo to y and starts servo
    public void wobbleGoals() {
        // set motor power to 1 when y is pressed
        if (gamepad1.y) {
            Rotate.set(ROTATE_POWER);
        }
        // starts front servo when b is pressed
        if (gamepad1.b) {
            frontHookServo = true;
        }
        // starts top servo when x is pressed
        if (gamepad1.x) {
            topHookServo = true;
        }
    }

    // binds shooter to left stick button .
    public void shooter() {
        // sets power to 1 when left stick button s pressed
        if (gamepad1.left_stick_button) {
            Shooter.set(SHOOTER_POWER);
        }
    }

    // binds intake to left bumper and sets power
    // binds reverse intake to right bumper and sets power
    // binds shooterServo to a and sets power
    public void intake() {
        // sets power to 1 when left bumper is pressed
        if (gamepad1.left_bumper) {
            Intake.set(INTAKE_POWER);
        }
        // sets power to -1 when right bumper is pressed
        if (gamepad1.right_bumper) {
            Intake.set(-INTAKE_POWER);
        }
        // starts servo when a is pushed
        if (gamepad1.a) {
            shooterServo = true;
        }
    }

    // binds stoppage of all motors/servos to dpad down
    public void stop() {
        // stops motors on robot
        if (gamepad1.dpad_down) {
            // stop non-drive motors
            Intake.set(0);
            Rotate.set(0);
            Shooter.set(0);

            // stop servos
            frontHookServo = false;
            topHookServo= false;
            shooterServo = true;

            // stop drive motors
            FrontLeftDrive.set(0);
            FrontRightDrive.set(0);
            BackLeftDrive.set(0);
            BackRightDrive.set(0);
        }
    }

    // drive according to controller inputs from sticks
    public void drive() {
        drive.driveRobotCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
    }
}