package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    private OUTTAKE_PIVOT_STATES outtakePivotState = OUTTAKE_PIVOT_STATES.REST;
    private LIFT_STATES liftState = LIFT_STATES.BOTTOM;
    private EXTENSION_STATES extensionState = EXTENSION_STATES.IN;
    private PIVOT_STATES pivotState = PIVOT_STATES.DEPOSIT;
    protected ElapsedTime time = new ElapsedTime();
    protected final Button a = new Button();
    protected final Button x = new Button();
    protected final Button b = new Button();
    protected final Button y = new Button();
    protected final Button gamepad2RightTriggerUp = new Button();
    protected final Button gamepad2RightTriggerDown = new Button();
    protected final Button dUp = new Button();
    protected final Button dDown = new Button();


    private double tappedTime = 0;

    protected enum CLAW_STATES {
        OPEN,
        CLOSED
    }

    protected enum OUTTAKE_PIVOT_STATES {
        REST,
        DEPOSIT
    }

    protected enum LIFT_STATES {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }

    protected enum PIVOT_STATES {
        COLLECT,
        HOLD,
        REVERSE_HOLD,
        DEPOSIT
    }

    protected enum EXTENSION_STATES {
        OUT,
        IN
    }


    protected double drivePower = 0.75;

    @Override
    public void init() {
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        robot.leftExtension.setDirection(Servo.Direction.REVERSE);
        robot.leftPivot.setDirection(Servo.Direction.REVERSE);
        robot.leftOuttakePivot.setDirection(Servo.Direction.REVERSE);
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

    protected void updateButtons() {
        a.update(gamepad2.a);
        x.update(gamepad2.x);
        y.update(gamepad2.y);
        b.update(gamepad2.b);
        gamepad2RightTriggerDown.update(gamepad2.right_stick_y > 0.1);
        gamepad2RightTriggerUp.update(gamepad2.right_stick_y < -0.1);
        dUp.update(gamepad2.dpad_up);
        dDown.update(gamepad2.dpad_down);
    }

    protected void manualServoSet(Button button, Servo servo, double position) {
        if (button.is(Button.States.TAP)) servo.setPosition(position);
    }

    protected void manualServoSet(Servo servo, double position) {
        servo.setPosition(position);
    }

    protected void extension() {
        if (dUp.is(Button.States.HELD) && robot.leftExtension.getPosition() < 0.45) {
            robot.leftExtension.setPosition(robot.leftExtension.getPosition() + 0.01);
            robot.rightExtension.setPosition(robot.rightExtension.getPosition() + 0.01);
        }
        if (dDown.is(Button.States.HELD) && robot.leftExtension.getPosition() > 0) {
            robot.leftExtension.setPosition(robot.leftExtension.getPosition() - 0.01);
            robot.rightExtension.setPosition(robot.rightExtension.getPosition() - 0.01);
        }
    }

    protected void pivot() {
        if (b.is(Button.States.TAP)) {
            if (pivotState == PIVOT_STATES.DEPOSIT) {
                robot.leftPivot.setPosition(0.25);
                robot.rightPivot.setPosition(0.25);
                pivotState = PIVOT_STATES.HOLD;
            } else if (pivotState == PIVOT_STATES.HOLD) {
                robot.leftPivot.setPosition(0.7);
                robot.rightPivot.setPosition(0.7);
                pivotState = PIVOT_STATES.COLLECT;
            } else if (pivotState == PIVOT_STATES.COLLECT) {
                robot.leftPivot.setPosition(0.25);
                robot.rightPivot.setPosition(0.25);
                pivotState = PIVOT_STATES.REVERSE_HOLD;
            } else if (pivotState == PIVOT_STATES.REVERSE_HOLD) {
                robot.leftPivot.setPosition(0);
                robot.rightPivot.setPosition(0);
                pivotState = PIVOT_STATES.DEPOSIT;
            }
        }
        dashboardTelemetry.addData("b", b.getState());
    }

    public void outtakePivot() {
        if (x.is(Button.States.TAP)) {
            if (outtakePivotState == OUTTAKE_PIVOT_STATES.REST) {
                robot.leftOuttakePivot.setPosition(0.6);
                robot.rightOuttakePivot.setPosition(0.6);
                outtakePivotState = OUTTAKE_PIVOT_STATES.DEPOSIT;
            } else if (outtakePivotState == OUTTAKE_PIVOT_STATES.DEPOSIT) {
                robot.leftOuttakePivot.setPosition(0);
                robot.rightOuttakePivot.setPosition(0);
                outtakePivotState = OUTTAKE_PIVOT_STATES.REST;
            }
        }
    }

    protected void intake() {
        if (y.is(Button.States.HELD)) {
            robot.intake.setPower(1);
        } else {
            robot.intake.setPower(0);
        }
    }

    protected void outtake() {

    }
}
