package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class AutoSuperOp extends OpMode {

    // delcare vuforia lisense key
    public final static String VUFORIA_KEY =
            "ARgYuCf/////AAABmUYfc1+dVEQsgUBCPA2kCAFRmuTRB/XUfAJzLsRyFDRg6uMMjj6EXM8YNiY5l3oTw83H"
                    + "+PKgfF46gctdzrln2nnVXMebpgN9ULy1cOfdSsPk0hwSZqzcY0LWCj+rPPrZ3JyQT7gf2aw7bo8ZvWedWB7skuGIjg"
                    + "+9cyTJdDyXmXrQ8Bo4r4siTFNTVFxg21OH/Gd8wrVJF4RqjE+kcez3MzcnE2EPCqWTNixSge5yLg+tN87/R"
                    + "/dMPzqHWvmjE6F6J/7/sahPt7FQ9G6tYWnV1impzZsH7T"
                    + "/JT6pGr2SALwHdaNjBGbYY76ZfvAxixEdob9g6qMBhKOyLg6HTP9VzRZ06ksUhErmR2K2LSkyjxBBz";
    public static final float mmPerInch = 25.4f;
    // declare labels for ring stacks and the model asset
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    // declare drive constants
    public final int cpr = 448;
    final int rpm = 64;
    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;
    // declare non-drive motors
    public Motor intake;

    /*
     * declare and initialize all booleans needed for Auto OpModes
     */
    public Motor rotate;
    public Motor shooter;
    // declare servos
    public ServoEx frontHook;
    public ServoEx topHook;
    public ServoEx shooterServo;
    // check if servo is going to position 1 or 0
    public boolean servoPos;
    // check if code has been in state STRAFETOTARGET & check that the angle is close to correct
    public boolean angleCorrect = false;
    // make sure each state only runs once (since all of our OpModes are iterative)
    public boolean lock = false;

    /*
     * declare and initialize all ints needed for Auto OpModes
     */
    // makes sure that the hesitation time only runs once
    public boolean shoot = false;
    // when there are zero rings, the code must go back to state ROTATECW, this boolean makes sure
    // the code run in the first instance of the state does not run again
    public boolean rotateZeroCW = false;
    // when there are zero rings, the code must go back to state ROTATECCW, this boolean makes sure
    // the code run in the first instance of the state does not run again
    public boolean rotateZeroCCW = false;
    // when there is one ring, the code must go back to state ROTATECW, this boolean makes sure
    // the code run in the first instance of the state does not run again
    public boolean rotateSingle = false;
    // when there are four rings, the code must go back to state ROTATECW, this boolean makes sure
    // the code run in the first instance of the state does not run again
    public boolean rotateQuad = false;
    // number of attempts to find nav servoPos
    public int turnCount = 0;

    /*
     * declare and initialize all doubles needed for Auto OpModes
     */
    // count number of servo movements
    public int servoMoveCount = 0;
    // 0 when adjusting angle the first time, 1 when adjusting angle the second time
    public int angleAdjustCount = 0;
    // 0 when checking for servoPos during rotation, 1 when angle adjusting, 2 when strafing, 3 when going back
    public int checkMoveType = 0;
    // keeps track of provided rings (for where to drop wobble goal)
    public int numRings = 0;
    // keep track of number of times code has been in park
    public int park = 0;
    // declare desiredY position (eg: about where the robot so be in the y direction on the field)
    // NOTE: the Y is actually horizontal, due to rev
    public double desiredY = 33;
    // declare desiredX position (eg: about where the robot so be in the x direction on the field)
    // NOTE: the X is actually vertical, due to rev
    public double desiredX = 44;
    // declare drive
    public MecanumDrive drive;
    // declare elapsed time
    public ElapsedTime time;
    // declare lastPos
    public ObjectLocator.RobotPos lastPos;
    // declare vuforia
    public VuforiaLocalizer vuforia;
    // declare tensorflow
    public TFObjectDetector tfod;
    public ObjectLocator objectLocator = null;

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
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);
        // set shooter to run with encoder (that way we can use velocity instead of the motor power)
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize servos
        topHook = new SimpleServo(hardwareMap, "TopHook");
        shooterServo = new SimpleServo(hardwareMap, "ShooterServo");

        // initialize drive
        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // initialize time
        time = new ElapsedTime();

        // call methods that initialize vuforia and tensorflow
        initVuforia();
        initTfod();

        // add drive telemetry
        telemetry.addData("Front Left Power", frontLeftDrive::get);
        telemetry.addData("Front Right Power", frontRightDrive::get);
        telemetry.addData("Back Left Power", backLeftDrive::get);
        telemetry.addData("Back Right Power", backRightDrive::get);
    }

    // get battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    // stop all drive motors, set RunMode to STOP_AND_RESET_ENCODER, then set to RUN_WITHOUT_ENCODER
    public void resetDrive() {
        drive.stop();
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // set drive motors to user specified RunMode (used in resetDrive() for clarity)
    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.motor.setMode(mode);
        frontRightDrive.motor.setMode(mode);
        backRightDrive.motor.setMode(mode);
        backLeftDrive.motor.setMode(mode);
    }

    // initialize vuforia
    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        CameraStreamServer.getInstance().setSource(vuforia);
    }

    // initialize tensorflow
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // detect rings
    public void detect() {
        // activate tensorflow
        tfod.activate();
        // set digital camera zoom
        tfod.setZoom(2.5, 16.0 / 9.0);

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            // loop through updatedRecognitions and print out telemetry
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                // if the list says there are four rings (LABEL_FIRST_ELEMENT)
                // set numRings to 4 so we can actually use this data
                // if the list says there is one ring (LABEL_SECOND_ELEMENT)
                // set numRings to 1
                // otherwise keep numRings at 0
                if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                    numRings = 4;
                    telemetry.addData("rings4", numRings);
                } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    numRings = 1;
                    telemetry.addData("rings1", numRings);
                }
            }
            telemetry.update();
        }
        telemetry.update();
    }
}

