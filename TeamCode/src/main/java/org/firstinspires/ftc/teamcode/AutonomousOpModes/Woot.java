package org.firstinspires.ftc.teamcode.AutonomousOpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Woot extends AutoSuperOp{
  public void init() {
      super.init();
  }

  public void loop() {
      drive.driveRobotCentric(0,0, -0.4);

      if (time.milliseconds() >= 1) {
          drive.stop();
          time.reset();
      }
  }
}
