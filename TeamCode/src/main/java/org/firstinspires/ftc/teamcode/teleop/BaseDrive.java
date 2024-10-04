package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;


@TeleOp(name="Base Drive", group="beta")
public class BaseDrive extends InheritableTeleOp {


    @Override
    public void loop() {
        drive(1);

        Utility.registerTelemetryImplementation(telemetryImplementations, () -> {
            dashboardTelemetry.addData("left stick y", gamepad1.left_stick_y);
            dashboardTelemetry.addData("left stick x", gamepad1.left_stick_x);
            dashboardTelemetry.addData("right stick x", gamepad1.right_stick_x);
        });

        updateTelemetry();

    }
}
