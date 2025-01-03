package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name="PID Test", group="test")
public class PIDTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    protected DcMotorEx motor = null;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        telemetry.addData("Status", "Initialized");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override

    public void loop() {


    }
}
