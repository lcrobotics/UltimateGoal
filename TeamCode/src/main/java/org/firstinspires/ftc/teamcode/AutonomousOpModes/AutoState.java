package org.firstinspires.ftc.teamcode.AutonomousOpModes;

public enum AutoState {
    DRIVEOVERMID(0),
    ROTATECW(1),
    FIXANGLE(2),
    STRAFETOTARGET(3),
    CORRECTX(4),
    ROTATECCW(5),
    SHOOT(6),
    DRIVETOMID(7),
    DONE(8),
    UPDATE(9),
    DRIVEBEHINDMID(10);

    int val;

    AutoState(int value) {
        val = value;
    }
}
