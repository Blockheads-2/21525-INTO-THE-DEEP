package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Button;


@TeleOp(name="Base Drive")
public class BaseDrive extends InheritableTeleOp {

    @Override
    public void start() {
        robot.leftExtension.setPosition(0);
        robot.rightExtension.setPosition(0);
        robot.leftPivot.setPosition(0);
        robot.rightPivot.setPosition(0);
    }

    @Override
    public void loop() {
        powerModifier();
        drive(drivePower);
        updateButtons();
        extension();
        pivot();
        intake();

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
