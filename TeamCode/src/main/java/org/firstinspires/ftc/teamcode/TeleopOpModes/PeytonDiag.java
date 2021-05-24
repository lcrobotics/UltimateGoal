package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


// A class used to show that the robot was wired incorrectly.
// Can be deleted
//@TeleOp
public class PeytonDiag extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    DcMotor fr;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotor.class, "BackRightDrive");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            fl.setPower(1);
        } else {
            fl.setPower(0);
        }
        if (gamepad1.b) {
            fr.setPower(1);
        } else {
            fr.setPower(0);
        }
        if (gamepad1.x) {
            bl.setPower(-1);
        } else {
            bl.setPower(0);
        }
        if (gamepad1.y) {
            br.setPower(-1);
        } else {
            br.setPower(0);
        }
        telemetry.addData("fl", fl.getPower());
        telemetry.addData("bl", bl.getPower());
        telemetry.addData("fr", fr.getPower());
        telemetry.addData("br", br.getPower());
    }
}
