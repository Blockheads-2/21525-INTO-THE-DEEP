package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Field Centric Drive", group="beta")
public class FieldCentricDrive extends InheritableTeleOp {
    public void loop() {
        powerModifier();
        drive(drivePower);

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

        dashboardTelemetry.update();
    }
}
