package org.firstinspires.ftc.teamcode.AutonomousOpModes;

/*
 * declare all states for auto OpModes
 * NOTE: not every state is used in each OpMode
 */

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
    DROPWOBBLE(12),
    TURNABIT(13),
    DRIVEABIT(14),
    DETECT(15),
    STRAFECW(16),
    ROTATECCWSTART(17),
    ROTATEWOBBLE(18),
    ROTATECWSTART(19),
    DRIVEFORWARDSTART(20);


    int val;

    AutoState(int value) {
        val = value;
    }
}
