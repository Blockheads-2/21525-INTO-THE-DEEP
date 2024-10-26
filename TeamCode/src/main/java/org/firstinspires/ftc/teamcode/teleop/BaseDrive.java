package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Button;


@TeleOp(name="Base Drive")
public class BaseDrive extends InheritableTeleOp {

    public void start() {
        robot.outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        powerModifier();
        drive(drivePower);
        lift();
        updateButtons();
        claw();
        clawAxial();

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());
        dashboardTelemetry.addData("lift velocity:", robot.outtakeSlide.getVelocity());
        dashboardTelemetry.addData("position", robot.outtakeSlide.getCurrentPosition());

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
