package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AccelDriveTest extends OpMode {
    DcMotor motor;
    AccelDrive drive;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        drive = new AccelDrive(motor);
    }


    boolean ran = false;
    @Override
    public void loop() {
        if (!ran) {
            drive.set(15000, .5, 1);
            ran = true;
        }

        drive.update();
        telemetry.addData("> current time: ", drive.currentTime);
        telemetry.addData("> current power:", drive.tempPower);
    }
}
