package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Base Drive")
public class Test extends InheritableTeleOp {

    public void start() {
    }

    @Override
    public void loop() {
        powerModifier();
        drive(drivePower);
        updateButtons();
        extension();

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

    }

    @Override
    public void stop() {
        super.stop();
    }
}
