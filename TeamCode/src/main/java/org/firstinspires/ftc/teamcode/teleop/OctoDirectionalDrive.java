package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Octo Directional Drive")
public class OctoDirectionalDrive extends InheritableTeleOp {
    @Override
    public void loop() {
        powerModifier();
        octoDirectionalDrive(drivePower);

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

        dashboardTelemetry.update();
    }
}

