package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import examples.VuforiaSuperOp;

public abstract class AutoSuperOp extends VuforiaSuperOp {
    final int cpr = 448;
    final int rpm = 64;
    //motors/servos for exactly what it says
    public Motor intake;
    public Motor rotate;
    public Motor shooter;
    public ServoEx frontHook;
    public ServoEx topHook;
    public ServoEx shooterServo;
    //wheels
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;

    public MecanumDrive drive;

    @Override
    public void init() {
        super.init();

        // Initializes each piece of hardware
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);
        frontHook = new SimpleServo(hardwareMap, "FrontHook");
        topHook = new SimpleServo(hardwareMap, "TopHook");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");

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

        // initialize drive
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        telemetry.addData("Front Left Power", frontLeftDrive::get);
        telemetry.addData("Front Right Power", frontRightDrive::get);
        telemetry.addData("Back Left Power", backLeftDrive::get);
        telemetry.addData("Back Right Power", backRightDrive::get);
    }
}
