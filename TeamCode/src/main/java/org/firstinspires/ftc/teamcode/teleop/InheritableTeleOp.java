package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public abstract class InheritableTeleOp extends OpMode {
    protected MecanumDrive robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    protected List<Runnable> telemetryImplementations = new ArrayList<>();

    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        updateTelemetry();
    }

    protected void drive(double power) {
        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        if (Math.abs(gamepad1.left_stick_x) > 0.25) directionX = Math.pow(gamepad1.left_stick_x, 1);
        if (Math.abs(gamepad1.left_stick_y) > 0.25)
            directionY = -Math.pow(gamepad1.left_stick_y, 1);
        if (Math.abs(gamepad1.right_stick_x) > 0.25)
            directionR = Math.pow(gamepad1.right_stick_x, 1);

        double leftFrontPower = (directionX + directionY + directionR) * power;
        double leftBackPower = (-directionX + directionY + directionR) * power;
        double rightFrontPower = (-directionX + directionY - directionR) * power;
        double rightBackPower = (directionX + directionY - directionR) * power;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);

        Utility.registerTelemetryImplementation(telemetryImplementations, () -> {
            dashboardTelemetry.addData("left front power:", leftFrontPower);
            dashboardTelemetry.addData("left back power:", leftBackPower);
            dashboardTelemetry.addData("right front power:", rightFrontPower);
            dashboardTelemetry.addData("right back power:", rightBackPower);

            dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
            dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
            dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
            dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());
        });
    }

    protected void simulateAnalogDrive(double gamepad1LeftStickX, double gamepad1LeftStickY, double gamepad1RightStickX, double power) {
        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        double leftFrontPower = (directionX + directionY + directionR) * power;
        double leftBackPower = (-directionX + directionY + directionR) * power;
        double rightFrontPower = (-directionX + directionY - directionR) * power;
        double rightBackPower = (directionX + directionY - directionR) * power;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);

        Utility.registerTelemetryImplementation(telemetryImplementations, () -> {
            dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
            dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
            dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
            dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());
        });
    }

    protected void powerModifier(double power) {

    }

    protected void updateTelemetry() {
        for (Runnable implementation : telemetryImplementations) {
            implementation.run();
        }

        dashboardTelemetry.update();
    }
}
