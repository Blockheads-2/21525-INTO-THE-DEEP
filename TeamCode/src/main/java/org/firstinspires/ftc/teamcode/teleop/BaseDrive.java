package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Base Drive", group="beta")
public class BaseDrive extends InheritableTeleOp {


    @Override
    public void loop() {
        drive(0.5);
        updateTelemetry();
    }
}
