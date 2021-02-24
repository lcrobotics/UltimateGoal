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

    // value used to make triggers buttons (for intake)
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

    // declare drive constructor
    public MecanumDrive drive;

    // frontHook booleans (for toggle)
    boolean frontOn = false;
    boolean isX = false;
    boolean wasX = false;

    // topHook booleans (for toggle)
    boolean topOn = false;
    boolean isA = false;
    boolean wasA = false;

    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;

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
        // reverse motor (mr. ross can't wire things)
        frontLeftDrive.setInverted(true);
        // multipliers on frontRightDrive and backLeftDrive are because of the weight imbalance on our robot
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, .7);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, .9);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        // reverse motor (mr. ross can't wire things)
        backRightDrive.setInverted(true);

        // initialize imu (needed for field centric driving)
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        // initialize drive (so we can drive)
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    // drive according to controller inputs from driver's sticks
    public void drive() {
        // call drive robot centric (meaning that the front is always the front, no matter where the robot is on the field)
        // multipliers slow down the robot so we don't run into things (it needs to be controlled)
        drive.driveRobotCentric(-gamepad1.left_stick_x * .8, -gamepad1.left_stick_y * .8, -gamepad1.right_stick_x * .8, true);
    }

    // binds intake to left trigger, reverse intake to right
    // both driver and operator can intake, but driver has precedence
    public void intake() {
        float intakePower = 0;
        // set intake as a button on right trigger and reverse intake on left trigger (button)
        if(gamepad2.right_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        }

        // set intake as a button on right trigger and reverse intake on left trigger (button)
        // and override operator
        if(gamepad1.right_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        }

        intake.set(intakePower);

        // triggers shooterServo when right bumper pressed
        if (gamepad1.right_bumper) {
            shooterServo.setPosition(0.6);
        } else {
            shooterServo.setPosition(0);
        }
    }

    // toggle shooter on driver's left bumper
    public void shooter() {
        // make left bumper toggle for shooter
        // track history of button
        if((isLB = gamepad1.left_bumper) && !wasLB) {
            if(shooterOn) {
                // if the shooter is on and left bumper is pressed, turn shooter off
                shooter.set(0);
            } else {
                // if the shooter is off and left bumper is pressed, turn shooter on
                shooter.set(-SHOOTER_POWER);
            }
            shooterOn = !shooterOn;
        }
        wasLB = isLB;
    }

    // bind rotate to operator's right stick
    // create toggle for front servo to operator's x
    // create toggle for top servo to operator's a
    public void wobbleGoals() {
        // bind rotate power to right stick of operator
        // multiplier slows motor down so it doesn't kill the robot
        rotate.set(gamepad2.right_stick_y * .4);

        // bind slow intake (for wobble goal locking in) to operator's left bumper
        if (gamepad2.left_bumper) {
            intake.set(-INTAKE_POWER_SLOW);
        } else {
            intake.set(0);
        }

        // toggles front servo on operator's x press
        if ((isX = gamepad2.x) && !wasX) {
            if(frontOn) {
                // if servo is open, close on x press
                frontHook.setPosition(0);
            } else {
                // if servo is closed, open on x press
                frontHook.setPosition(1);
            }
            frontOn = !frontOn;
        }
        wasX = isX;

        // toggles top servo on operator's a press
        if((isA = gamepad2.a) && !wasA) {
            if(topOn) {
                // if servo is open, close on a press
                topHook.setPosition(0);
            } else {
                // if servo is closed, open on a press
                topHook.setPosition(1);
            }
            topOn = !topOn;
        }
        wasA = isA;
    }

    // bind stoppage of motors/servos that each person controls to dpad down
    public void stop() {
        // if driver presses dpad down, stop motors they control
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

        // if operator presses dpad down, stop motors/servos they control
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
}