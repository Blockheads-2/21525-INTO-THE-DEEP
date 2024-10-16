package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Button;


@TeleOp(name="Base Drive")
public class BaseDrive extends InheritableTeleOp {
    private final Button a = new Button();

    @Override
    public void loop() {
        a.update(gamepad1.a);
        powerModifier();
        drive(drivePower);
        manualServoSet(a, robot.claw, 0.1);

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        manualServoSet(robot.claw, 0);

        super.stop();
    }
}
