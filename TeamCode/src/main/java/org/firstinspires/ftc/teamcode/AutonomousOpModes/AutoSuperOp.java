package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.util.ElapsedTime;

import examples.VuforiaSuperOp;

public abstract class AutoSuperOp extends VuforiaSuperOp {
    // declare drive constants
    final int cpr = 448;
    final int rpm = 64;

    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;

    // declare non-drive motors
    public Motor intake;
    public Motor rotate;
    public Motor shooter;

    // declare servos
    public ServoEx frontHook;
    public ServoEx topHook;
    public ServoEx shooterServo;

    // check if servo is going to 1 or 0
    boolean target;
    // check if code has been in state SIDEWAYS & check that the angle is close to correct
    boolean strafeAngle = false;
    // boolean to make sure that nothing runs 40 times
    boolean lock = false;

    // number of attempts to find nav target
    int rotNum = 0;
    // count number
    int ringsShot = 0;
    // 0 when adjusting angle the first time, 1 when adjusting angle the second time
    int rot = 0;
    // 0 when checking for target during rotation, 1 when angle adjusting, 2 when strafing, 3 when going back
    int checkState = 0;

    // declare drive
    public MecanumDrive drive;

    // declare elapsed time
    ElapsedTime time;

    // declare lastPos
    ObjectLocator.RobotPos lastPos;

    @Override
    public void init() {
        super.init();
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setInverted(true);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setInverted(true);

        // initialize non-drive motors
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);

        // initialize servos
        frontHook = new SimpleServo(hardwareMap, "FrontHook");
        topHook = new SimpleServo(hardwareMap, "TopHook");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");

        // initialize drive
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // initialize time
        time = new ElapsedTime();

        // add drive telemetry
        telemetry.addData("Front Left Power", frontLeftDrive::get);
        telemetry.addData("Front Right Power", frontRightDrive::get);
        telemetry.addData("Back Left Power", backLeftDrive::get);
        telemetry.addData("Back Right Power", backRightDrive::get);
    }
}
