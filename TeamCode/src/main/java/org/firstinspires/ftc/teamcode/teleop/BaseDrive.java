package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;


@TeleOp(name="Base Drive", group="beta")
public class BaseDrive extends InheritableTeleOp {


    @Override
    public void loop() {
        drive(0.5);
        updateTelemetry();
    }
}
