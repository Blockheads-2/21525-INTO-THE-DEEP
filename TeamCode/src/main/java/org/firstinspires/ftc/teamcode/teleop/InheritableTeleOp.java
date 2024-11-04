package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.roadrunner.MecanumDrive;

public abstract class InheritableTeleOp extends OpMode {
    protected MecanumDrive robot;
    protected FtcDashboard dashboard;
    protected Telemetry dashboardTelemetry;
    private CLAW_STATES clawState = CLAW_STATES.CLOSED;
    private CLAW_AXIAL_STATES clawAxialState = CLAW_AXIAL_STATES.REST;
    private LIFT_STATES liftState = LIFT_STATES.BOTTOM;
    protected ElapsedTime time = new ElapsedTime();
    protected final Button rightStickUp = new Button();
    protected final Button rightStickDown = new Button();
    protected final Button a = new Button();
    protected final Button x = new Button();

    private double tappedTime = 0;

    protected enum CLAW_STATES {
        OPEN,
        CLOSED
    }

    protected enum CLAW_AXIAL_STATES {
        REST,
        REVERSE,
        DOWN
    }

    protected enum LIFT_STATES {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }

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

    protected void claw() {
        if (a.is(Button.States.TAP)) {
            if (clawState == CLAW_STATES.CLOSED) {
                robot.claw.setDirection(Servo.Direction.FORWARD);
                robot.claw.setPosition(0.1);
                clawState = CLAW_STATES.OPEN;
            } else if (clawState == CLAW_STATES.OPEN) {
                robot.claw.setDirection(Servo.Direction.REVERSE);
                robot.claw.setPosition(0);
                clawState = CLAW_STATES.CLOSED;
            }
        }

        telemetry.addData("thing", clawState);
        telemetry.addData("pos", robot.claw.getPosition());
        telemetry.update();
    }

    protected void clawAxial() {
//        if (x.is(Button.States.TAP)) {
//            if (clawAxialState == CLAW_AXIAL_STATES.REST) {
//                robot.clawAxial.setDirection(Servo.Direction.REVERSE);
//                robot.clawAxial.setPosition(1);
//                clawAxialState = CLAW_AXIAL_STATES.REVERSE;
//            } else if (clawAxialState == CLAW_AXIAL_STATES.REVERSE) {
//                robot.clawAxial.setDirection(Servo.Direction.REVERSE);
//                robot.clawAxial.setPosition(1);
//                clawAxialState = CLAW_AXIAL_STATES.DOWN;
//            } else if (clawAxialState == CLAW_AXIAL_STATES.DOWN) {
//                robot.clawAxial.setDirection(Servo.Direction.FORWARD);
//                robot.clawAxial.setPosition(0);
//                clawAxialState = CLAW_AXIAL_STATES.REST;
//            }
//        }
        robot.clawAxial.setPower(-gamepad2.left_stick_y);
    }


    protected void updateButtons() {
        rightStickDown.update(gamepad2.right_stick_y > 0);
        rightStickUp.update(gamepad2.right_stick_y < 0);
        a.update(gamepad2.a);
        x.update(gamepad2.x);
    }

    protected void manualServoSet(Button button, Servo servo, double position) {
        if (button.is(Button.States.TAP)) servo.setPosition(position);
    }

    protected void manualServoSet(Servo servo, double position) {
        servo.setPosition(position);
    }
    protected void lift() {
        if (rightStickUp.is(Button.States.TAP)) {

            if (liftState == LIFT_STATES.MIDDLE) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.TOP);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.75);
                liftState = LIFT_STATES.TOP;
            }
            if (liftState == LIFT_STATES.LOW) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.MIDDLE);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.75);
                liftState = LIFT_STATES.MIDDLE;
            }
            if (liftState == LIFT_STATES.BOTTOM) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.LOW);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.75);
                liftState = LIFT_STATES.LOW;
            }
        }
        if (rightStickDown.is(Button.States.TAP)) {

            if (liftState == LIFT_STATES.LOW) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.BOTTOM);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.25);
                liftState = LIFT_STATES.BOTTOM;
            }
            if (liftState == LIFT_STATES.MIDDLE) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.LOW);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.25);
                liftState = LIFT_STATES.LOW;
            }
            if (liftState == LIFT_STATES.TOP) {
                robot.outtakeSlide.setTargetPosition((int) Constants.Lift.MIDDLE);
                robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.outtakeSlide.setPower(0.25);
                liftState = LIFT_STATES.MIDDLE;
            }
        }
    }

    protected void extendo() {

    }
}
