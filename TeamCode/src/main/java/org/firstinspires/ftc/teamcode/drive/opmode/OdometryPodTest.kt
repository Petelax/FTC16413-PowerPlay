package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class OdometryPodTest : OpMode() {
    lateinit var leftOdo : Motor
    lateinit var strafeOdo : Motor
    lateinit var rightOdo : Motor

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        leftOdo = Motor(hardwareMap, "odoPod0") /* left odo pod */
        strafeOdo = Motor(hardwareMap, "elevator0") /* strafe odo pod */
        rightOdo = Motor(hardwareMap, "bL") /* right odo pod */

        leftOdo.encoder.setDirection(Motor.Direction.REVERSE)
        rightOdo.encoder.setDirection(Motor.Direction.REVERSE)

        leftOdo.resetEncoder()
        strafeOdo.resetEncoder()
        rightOdo.resetEncoder()
    }

    override fun loop() {
        telemetry.addData("leftOdo", leftOdo.currentPosition)
        telemetry.addData("strafeOdo", strafeOdo.currentPosition)
        telemetry.addData("rightOdo", rightOdo.currentPosition)
        telemetry.update()
    }
}