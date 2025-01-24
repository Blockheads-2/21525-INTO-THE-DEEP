package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


abstract class InheritableAutonomous : LinearOpMode() {
    override fun runOpMode() {}
}

class Lift(hardwareMap: HardwareMap) {
    private val lift: DcMotorEx

    init {
        lift = hardwareMap.get(DcMotorEx::class.java, "rightLift")
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_TO_POSITION
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.direction = DcMotorSimple.Direction.FORWARD
    }
}

class Extension(hardwareMap: HardwareMap) {
    private val leftExtension: Servo
    private val rightExtension: Servo

    init {
        leftExtension = hardwareMap.get(Servo::class.java, "leftExtension")
        rightExtension = hardwareMap.get(Servo::class.java, "rightExtension")
        leftExtension.direction = Servo.Direction.REVERSE
        leftExtension.position = 0.0;
        rightExtension.position = 0.0;
    }
}

class Pivot(hardwareMap: HardwareMap) {
    private val leftPivot: Servo
    private val rightPivot: Servo

    init {
        leftPivot = hardwareMap.get(Servo::class.java, "leftPivot")
        rightPivot = hardwareMap.get(Servo::class.java, "rightPivot")
        rightPivot.direction = Servo.Direction.REVERSE
        leftPivot.position = 0.0;
        rightPivot.position = 0.0;
    }
}

class Claw(hardwareMap: HardwareMap) {
    private val claw: Servo

    init {
        claw = hardwareMap.get(Servo::class.java, "claw")
        claw.position = 0.0;
    }
}

class Intake(hardwareMap : HardwareMap) {
    private val intake: CRServo

    init {
        intake = hardwareMap.get(CRServo::class.java, "intake")

    }
}