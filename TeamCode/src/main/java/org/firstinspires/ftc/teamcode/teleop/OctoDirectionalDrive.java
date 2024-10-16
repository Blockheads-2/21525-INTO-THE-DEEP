package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Button;

@TeleOp(name="Octo Directional Drive", group="beta")
public class OctoDirectionalDrive extends InheritableTeleOp {
    private final Button a = new Button();

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

