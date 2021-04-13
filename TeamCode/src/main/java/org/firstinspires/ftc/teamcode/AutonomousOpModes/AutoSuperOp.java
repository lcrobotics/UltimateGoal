package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.commandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.hardware.ServoEx;
import com.lcrobotics.easyftclib.commandCenter.hardware.SimpleServo;
import com.lcrobotics.easyftclib.vision.ObjectLocator;
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

import examples.VuforiaSuperOp;

public abstract class AutoSuperOp extends VuforiaSuperOp {

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
    public Motor rotate;
    public Motor shooter;

    // declare servos
    public ServoEx frontHook;
    public ServoEx topHook;
    public ServoEx shooterServo;

    /*
     * declare and initialize all booleans needed for Auto OpModes
     */

    // check if servo is going to position 1 or 0
    public boolean servoPos;
    // check if code has been in state STRAFETOTARGET & check that the angle is close to correct
    public boolean angleCorrect = false;
    // boolean to make sure that nothing runs 40 times
    public boolean lock = false;
    // in some turning states (TURNABIT, ROTATECW, and ROTATECCW), some OpModes need to be there
    // twice with different times/ending states, use to make sure that works
    public boolean turn = false;
    // makes sure that the hesitation time only runs once
    public boolean shoot = false;

    /*
     * declare and initialize all ints needed for Auto OpModes
     */

    // number of attempts to find nav servoPos
    public int turnCount = 0;
    // count number of servo movements
    public int servoMoveCount = 0;
    // 0 when adjusting angle the first time, 1 when adjusting angle the second time
    public int angleAdjustCount = 0;
    // 0 when checking for servoPos during rotation, 1 when angle adjusting, 2 when strafing, 3 when going back
    public int checkMoveType = 0;
    // keeps track of provided rings (for where to drop wobble goal)
    public int numberRings = 4;
    // keep track of number of times code has been in park
    public int park = 0;

    /*
     * declare and initialize all doubles needed for Auto OpModes
     */

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

    @Override
    public void init() {
        super.init();
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);
        // set zero behavior to brake - so that the drive stops right away
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // reverse motor because Mr. Ross can't wire things
        frontLeftDrive.setInverted(true);
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        // set zero behavior to brake - so that the drive stops right away
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        // set zero behavior to brake - so that the drive stops right away
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        // set zero behavior to brake - so that the drive stops right away
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // reverse motor because Mr. Ross can't wire things
        backRightDrive.setInverted(true);

        // initialize non-drive motors
        intake = new Motor(hardwareMap, "Intake", cpr, rpm);
        rotate = new Motor(hardwareMap, "Rotate", cpr, rpm);
        shooter = new Motor(hardwareMap, "Shooter", cpr, rpm);
        // set shooter to run with encoder (that way we can use velocity instead of the motor power)
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    /*public void initVuforia() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        CameraStreamServer.getInstance().setSource(vuforia);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void detect() {
        tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 1.78 or 16/9).
        tfod.setZoom(2.5, 16.0 / 9.0);

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List <Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
            telemetry.update();
        }
        //tfod.shutdown();
        telemetry.update();
    }
    */
}

