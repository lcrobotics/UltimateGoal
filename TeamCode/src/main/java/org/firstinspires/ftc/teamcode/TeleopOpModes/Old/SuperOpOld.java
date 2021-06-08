package org.firstinspires.ftc.teamcode.TeleopOpModes.Old;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class SuperOpOld extends OpMode {
    // power constants
    final double INTAKE_POWER = 1;
    final double INTAKE_POWER_SLOW = .425;
    final float SHOOTER_POWER = 1f;

    // value used to make triggers buttons (for intake)
    final double THRESHOLD = 0.12;

    // drive constants
    final int cpr = 448;
    final int rpm = 64;
    // declare drive constructor
    public MecanumDrive drive;
    // declare drive motors
    Motor frontLeftDrive;
    Motor frontRightDrive;
    Motor backLeftDrive;
    Motor backRightDrive;
    // declare non-drive motors
    Motor intake;
    Motor intake2;
    Motor rotate;
    Motor shooter;
    // declare servos
    ServoEx frontHook;
    ServoEx vertical;
    ServoEx topHook;
    ServoEx shooterServo;
    ServoEx shooterControl;

    ServoEx wobbleLock;

    // frontHook booleans (for toggle)
    boolean frontOn = false;
    boolean isA = false;
    boolean wasA = false;

    // topHook booleans (for toggle)
    boolean topOn = false;
    boolean isY = false;
    boolean wasY = false;


    // stick booleans (for toggle)
    int reverse = -1;
    boolean isStick = false;
    boolean wasStick = false;


    // shooter booleans (for toggle)
    boolean shooterOn = false;
    boolean isLB = false;
    boolean wasLB = false;

    boolean isDpad = false;
    boolean wasDpad = false;

    @Override
    public void init() {
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm, 0.9);

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // multipliers on frontRightDrive and backLeftDrive are because of the weight imbalance on our robot
        // was .6
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, 0.9);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setInverted(true);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, 1);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm, 1);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // initialize non-drive motors
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        intake2 = new Motor(hardwareMap, "Intake2", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);

        wobbleLock = new SimpleServo(hardwareMap, "WobbleLock");

        // set shooter to float so that we don't murder another motor
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // initialize servos
        frontHook = new SimpleServo(hardwareMap, "FrontHook");
        topHook = new SimpleServo(hardwareMap, "TopHook");
        vertical = new SimpleServo(hardwareMap, "Vertical");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");
        shooterControl = new SimpleServo(hardwareMap, "ShooterControl");

        // initialize drive (so we can drive)
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        drive.setMaxSpeed(1);

        // add drive telemetry
        telemetry.addData("Front Left Power", frontLeftDrive::get);
        telemetry.addData("Front Right Power", frontRightDrive::get);
        telemetry.addData("Back Left Power", backLeftDrive::get);
        telemetry.addData("Back Right Power", backRightDrive::get);
        telemetry.addData("Shooter Velocity", this::getShooterVelocity);
    }

    double getShooterVelocity() {
        return ((DcMotorEx)shooter.motor).getVelocity(AngleUnit.DEGREES) / 360 * 60;
    }
    // drive according to controller inputs from driver's sticks
    public void drive() {
        double strafePower = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
        double forwardPower = Math.abs(gamepad1.left_stick_y) < 0.1 ? 0 : gamepad1.left_stick_y;
        double turnPower = Math.abs(gamepad1.right_stick_x) < 0.1 ? 0 : gamepad1.right_stick_x;

        if (((isStick = gamepad1.left_stick_button) && !wasStick)) {
            reverse *= -1;
        }

        drive.driveRobotCentric(
                strafePower * 0.8 * reverse,
                forwardPower * 0.8 * reverse,
                turnPower * 0.7,
                true
        );
        wasStick = isStick;

    }

    // binds intake to left trigger, reverse intake to right
    // both driver and operator can intake, but driver has precedence
    public void intake() {
        double intakePower = 0;

        if (gamepad2.left_bumper) {
            intakePower = -INTAKE_POWER_SLOW;
        } else if (gamepad2.right_bumper) {
            intakePower = INTAKE_POWER_SLOW;
        }

        // set intake as a button on right trigger and reverse intake on left trigger (button)
        if (gamepad2.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad2.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

        // set intake as a button on right trigger and reverse intake on left trigger (button)
        // and override operator
        if (gamepad1.right_trigger > THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.left_trigger > THRESHOLD) {
            intakePower = -INTAKE_POWER;
        }

//        if (Math.abs(gamepad2.left_stick_y) > THRESHOLD) {
//            intakePower = gamepad2.left_stick_y * 0.5;
//        }


        intake.set(intakePower);
        intake2.set(-intakePower);

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
        if ((isLB = gamepad1.left_bumper) && !wasLB) {
            if (shooterOn) {
                // if the shooter is on and left bumper is pressed, turn shooter off
                shooter.set(0);
            } else {
                // if the shooter is off and left bumper is pressed, turn shooter on
                ((DcMotorEx) shooter.motor).setVelocity(shooter.motor.getMotorType().getAchieveableMaxTicksPerSecond());
            }
            shooterOn = !shooterOn;
        }
        wasLB = isLB;
    }


    final double wobbleLockOpenPosition = .2;
    final double wobbleLockClosePosition = 0;


    boolean wobbleLockActuated = false;
    boolean lastGamepad2Bpress = false;
    // bind rotate to operator's right stick
    // create toggle for front servo to operator's x
    // create toggle for top servo to operator's a
    public void wobbleGoals() {


        // toggles front servo on operator's x press
        if ((isA = gamepad2.a) && !wasA) {
            if (frontOn) {
                // if servo is open, close on x press
                frontHook.setPosition(.93);
            } else {
                // if servo is closed, open on x press
                frontHook.setPosition(.3);
            }
            frontOn = !frontOn;
        }
        wasA = isA;

        // toggles top servo on operator's a press
        if ((isY = gamepad2.y) && !wasY) {
            if (topOn) {
                // if servo is open, close on a press
                topHook.setPosition(0);
            } else {
                // if servo is closed, open on a press
                topHook.setPosition(0.5);
            }
            topOn = !topOn;
        }
        wasY = isY;

        // bind rotate power to right stick of operator
        // multiplier slows motor down so it doesn't kill the robot
        // if wobbleArm is moving up, disengage lock
        double temp = gamepad2.right_stick_y;
        rotate.set(temp * 0.35);
        if (temp < -.86) {
//            wobbleLockActuated = false;
        } else if (!lastGamepad2Bpress && gamepad2.b) {
            wobbleLockActuated = !wobbleLockActuated;
        }

        wobbleLock.setPosition(wobbleLockActuated ? wobbleLockClosePosition : wobbleLockOpenPosition);
        lastGamepad2Bpress = gamepad2.b;
    }


    // bind stoppage of motors/servos that each person controls to dpad down
    public void driverStop() {
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