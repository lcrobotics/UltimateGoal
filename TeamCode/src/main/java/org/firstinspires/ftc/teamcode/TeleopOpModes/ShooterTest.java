package org.firstinspires.ftc.teamcode.TeleopOpModes;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;


@TeleOp
public class ShooterTest extends OpMode {
    Motor shooter;
    // declare motor constants
    final int cpr = 448;
    final int rpm = 64;
    final int portNumber = 1;

    DcMotorImplEx implEx;
    DcMotorController controller;

    public void init() {
        shooter = new Motor(hardwareMap, "shoot", cpr, rpm);
        implEx = new DcMotorImplEx(controller, portNumber, FORWARD);     }

    public void loop() {
        if(gamepad1.a) {
            implEx.setMotorEnable();
        } else {
            implEx.setMotorDisable();
        }
    }
}
