package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;


@TeleOp(name="Simulate", group="beta")
public class Simulate extends InheritableTeleOp {


    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.rightFront.setPower(0.5);
            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(0.5);
            robot.rightBack.setPower(0.5);
        } else {
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }
}
