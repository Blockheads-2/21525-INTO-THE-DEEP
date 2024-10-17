package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

public abstract class InheritableTeleOp extends OpMode {
    protected MecanumDrive robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;

    protected double drivePower = 0.75;

    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.update();
    }

    public void loop() {
        dashboardTelemetry.update();
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
    }

    protected void octoDirectionalDrive(double power) {
        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        if (Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y) > 0.25) {

            double directionAngle = Math.round(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / (Math.PI / 4)) * (Math.PI / 4);

            directionX = Math.cos(directionAngle);
            directionY = Math.sin(directionAngle);
        }
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
    }

    protected void fieldCentricDrive(double power) {
        double heading = robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw();

        double directionX = 0;
        double directionY = 0;
        double directionR = 0;

        if (Math.abs(gamepad1.left_stick_x) > 0.25) directionX = Math.pow(gamepad1.left_stick_x, 1);
        if (Math.abs(gamepad1.left_stick_y) > 0.25)
            directionY = -Math.pow(gamepad1.left_stick_y, 1);
        if (Math.abs(gamepad1.right_stick_x) > 0.25)
            directionR = Math.pow(gamepad1.right_stick_x, 1);

        double rotationalX = directionX * Math.cos(-heading) - directionY * Math.sin(-heading);
        double rotationalY = directionX * Math.cos(-heading) + directionY * Math.sin(-heading);

        rotationalX *= 1.1;

        double leftFrontPower = (rotationalX + rotationalY + directionR) * power;
        double leftBackPower = (-rotationalX + rotationalY + directionR) * power;
        double rightFrontPower = (-rotationalX + rotationalY - directionR) * power;
        double rightBackPower = (rotationalX + rotationalY - directionR) * power;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightBack.setPower(rightBackPower);
    }

    protected void powerModifier() {
        if (gamepad1.right_bumper) drivePower = 0.25;
        else if (gamepad1.right_trigger > 0.25) drivePower = 1;
        else drivePower = 0.75;
    }

    protected void manualServoSet(Button button, Servo servo, double position) {
        if (button.is(Button.States.TAP)) servo.setPosition(position);
    }

    protected void manualServoSet(Servo servo, double position) {
        servo.setPosition(position);
    }
}
