package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Simulate TeleOp Drive", group="beta")
public class SimulateTeleOpDrive extends InheritableTeleOp {

    @Override
    public void loop() {
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        updateTelemetry();
    }
}
