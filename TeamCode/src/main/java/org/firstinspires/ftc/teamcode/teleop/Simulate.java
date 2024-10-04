package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;


@TeleOp(name="Base Drive", group="beta")
public class Simulate extends InheritableTeleOp {


    @Override
    public void loop() {
        drive(1);


    }
}
