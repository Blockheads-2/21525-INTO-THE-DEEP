package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;


@TeleOp(name="Base Drive", group="Beta")
public class BaseDrive extends OpMode {
    MecanumDrive robot;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    double drivePower = 0.5;
    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        if (Math.abs(gamepad1.left_stick_x) > 0.25)
            directionX = Math.pow(gamepad1.left_stick_x, 1);
        if (Math.abs(gamepad1.left_stick_y) > 0.25)
            directionY = -Math.pow(gamepad1.left_stick_y, 1);
        if (Math.abs(gamepad1.right_stick_x) > 0.25)
            directionR = Math.pow(gamepad1.right_stick_x, 1);

        double leftFrontPower = (directionX + directionY + directionR) * drivePower;
        double leftBackPower = (-directionX + directionY + directionR) * drivePower;
        double rightFrontPower = (-directionX + directionY - directionR) * drivePower;
        double rightBackPower = (directionX + directionY - directionR) * drivePower;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);

        dashboardTelemetry.addData("left front power:", leftFrontPower);
        dashboardTelemetry.addData("left back power:", leftBackPower);
        dashboardTelemetry.addData("right front power:", rightFrontPower);
        dashboardTelemetry.addData("right back power:", rightBackPower);

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

        dashboardTelemetry.update();
    }
}
