package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class IndependentBaseDrive extends OpMode {
    protected MecanumDrive robot;

    double power = 0.5;
    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        if (Math.abs(gamepad1.left_stick_x) > 0.25) directionX = Math.pow(gamepad1.left_stick_x, 1);
        if (Math.abs(gamepad1.left_stick_y) > 0.25)
            directionY = -Math.pow(gamepad1.left_stick_y, 1);
        if (Math.abs(gamepad1.right_stick_x) > 0.25)
            directionR = Math.pow(gamepad1.right_stick_x, 1);

        double leftFrontPower = (directionX + directionY + directionR) * power;
        double leftBackPower = -(-directionX + directionY + directionR) * power;
        double rightFrontPower = (-directionX + directionY - directionR) * power;
        double rightBackPower = -(directionX + directionY - directionR) * power;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);
    }
}
