package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;

@TeleOp(name="Simulate TeleOp Drive", group="beta")
public class SimulateTeleOpDrive extends InheritableTeleOp {

    @Override
    public void loop() {
        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);
        robot.rightFront.setPower(1);
        robot.rightBack.setPower(1);

        Utility.registerTelemetryImplementation(telemetryImplementations, () -> {
            dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
            dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
            dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
            dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());
        });

        updateTelemetry();
    }
}
