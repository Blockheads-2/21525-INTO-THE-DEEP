package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Simulate TeleOp Drive", group="beta")
public class SimulateTeleOpDrive extends InheritableTeleOp {

    @Override
    public void loop() {
        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);
        updateTelemetry();
    }
}
