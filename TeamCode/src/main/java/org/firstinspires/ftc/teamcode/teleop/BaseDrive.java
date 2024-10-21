package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Button;


@TeleOp(name="Base Drive")
public class BaseDrive extends InheritableTeleOp {

    @Override
    public void loop() {
//        updateButtons();
        powerModifier();
//        claw();
        drive(drivePower);
        lift();

        dashboardTelemetry.addData("left front velocity:", robot.leftFront.getVelocity());
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.getVelocity());
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.getVelocity());
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.getVelocity());
        dashboardTelemetry.addData("lift velocity:", robot.outtakeSlide.getVelocity());

        dashboardTelemetry.update();
//        telemetry.addData("button state:", clawState);
//        telemetry.addData("button:", a.getState());
//        telemetry.update();
    }

    @Override
    public void stop() {
//        manualServoSet(robot.claw, 0);

        super.stop();
    }
}
