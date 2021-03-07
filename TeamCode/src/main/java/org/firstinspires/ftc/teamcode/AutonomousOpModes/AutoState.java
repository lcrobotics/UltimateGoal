package org.firstinspires.ftc.teamcode.AutonomousOpModes;

public enum AutoState {
    DRIVEOVERMID(0),
    ROTATECCW(1),
    FIXANGLE(2),
    STRAFETOTARGET(3),
    CORRECTX(4),
    ROTATECW(5),
    SHOOT(6),
    DRIVETOMID(7),
    DONE(8),
    UPDATE(9),
    DRIVEBEHINDMID(10),
    CENTER(11),
    DROPWOBBLE(12);

    int val;

    AutoState(int value) {
        val = value;
    }
}
