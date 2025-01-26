package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Base Drive")
class BaseDrive : InheritableTeleOp() {
    override fun start() {
        robot.leftExtension.position = 0.0
        robot.rightExtension.position = 0.0
        robot.leftPivot.position = 0.0
        robot.rightPivot.position = 0.0
        robot.leftOuttakePivot.position = 0.0
        robot.rightOuttakePivot.position = 0.0
        robot.claw.position = 0.0
    }

    override fun loop() {
        powerModifier()
        drive(drivePower)
        updateButtons()
        extension()
        pivot()
        intake()
        outtakePivot()
        lift()
        claw()
        dashboardTelemetry.addData("left front velocity:", robot.leftFront.velocity)
        dashboardTelemetry.addData("left back velocity:", robot.leftBack.velocity)
        dashboardTelemetry.addData("right front velocity:", robot.rightFront.velocity)
        dashboardTelemetry.addData("right back velocity:", robot.rightBack.velocity)
        dashboardTelemetry.addData("left front power:", robot.leftFront.power)
        dashboardTelemetry.addData("left back power:", robot.leftBack.power)
        dashboardTelemetry.addData("right front power:", robot.rightFront.power)
        dashboardTelemetry.addData("right back power:", robot.rightBack.power)
        dashboardTelemetry.update()
    }

    override fun stop() {
        super.stop()
    }
}
