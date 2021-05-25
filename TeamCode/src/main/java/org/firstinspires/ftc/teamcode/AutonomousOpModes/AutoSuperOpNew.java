package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.lcrobotics.easyftclib.vision.ObjectLocator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class AutoSuperOpNew extends OpMode {

    // declare vuforia license key
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
    public Motor carousel;
    public Motor shooter;
    public Motor intake;
    public Motor wobbleRotate;

    // declare servos
    public ServoEx autoWobble;

    // declare touch sensor
    public TouchSensor touchSensor;

    /* declare all booleans needed for auto opmodes */
    // make sure each state only runs once (since all of our OpModes are iterative)
    public boolean lock = false;

    /* declare all ints needed for auto opmodes */
    // keeps track of provided rings (for where to drop wobble goal)
    public int numRings = 0;

    /* declare all constructors needed for auto opmodes */

    // declare drive
    public MecanumDrive drive;
    // declare time
    public ElapsedTime time;
    // declare vuforia
    public VuforiaLocalizer vuforia;
    // declare tfod
    public TFObjectDetector tfod;

    @Override
    public void init() {
        // initialize drive motors
        // NOTE: scalars on frontLeftDrive and frontRightDrive are due to robot weight imbalance
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm, 0.9);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm, 0.9);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm, 1);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm, 1);

        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // initialize non-drive motors
        carousel = new Motor(hardwareMap, "carousel", cpr, rpm);
        shooter = new Motor(hardwareMap, "shooter", cpr, rpm);
        intake = new Motor(hardwareMap, "intake", cpr, rpm);
        wobbleRotate = new Motor(hardwareMap, "wobbleRotate", cpr, rpm);
        // set shooter to run with encoder (that way we can use velocity instead of the motor power)

        // initialize servos
        autoWobble = new SimpleServo(hardwareMap, "autoWobble");

        // initialize TouchSensor
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

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