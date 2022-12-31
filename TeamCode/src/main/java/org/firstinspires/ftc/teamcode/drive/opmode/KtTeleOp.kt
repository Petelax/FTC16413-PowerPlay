package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.Toggle
import org.firstinspires.ftc.teamcode.util.TwoButtonToggle
import org.openftc.easyopencv.OpenCvCamera

@TeleOp
class KtTeleOp: OpMode() {
    private lateinit var fL: Motor; lateinit var fR : Motor; lateinit var bL : Motor; lateinit var bR : Motor;
    private lateinit var elevator0: Motor; lateinit var elevator1 : Motor; lateinit var arm : Motor;

    private lateinit var intake: SimpleServo
    private lateinit var drive: MecanumDrive
    private lateinit var elevator: MotorGroup
    private lateinit var gamepadEx1: GamepadEx; lateinit var gamepadEx2: GamepadEx
    private lateinit var gyro: RevIMU

    private lateinit var intakeToggle: Toggle
    private lateinit var driveToggle: TwoButtonToggle

    private val gyroOffset: Double = 0.0


    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        fL = Motor(hardwareMap, "fL")
        fR = Motor(hardwareMap, "fR")
        bL = Motor(hardwareMap, "bL")
        bR = Motor(hardwareMap, "bR")
        elevator0 = Motor(hardwareMap, "elevator0", Motor.GoBILDA.RPM_435)
        elevator1 = Motor(hardwareMap, "elevator1", Motor.GoBILDA.RPM_435)
        arm = Motor(hardwareMap, "arm")
        intake = SimpleServo(hardwareMap, "intake", -135.0, 135.0, AngleUnit.DEGREES)


        fL.resetEncoder()
        fR.resetEncoder()
        bL.resetEncoder()
        bR.resetEncoder()


        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        elevator1.inverted = true
        elevator = MotorGroup(elevator0, elevator1)
        elevator.resetEncoder()
        elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        drive = MecanumDrive(fL, fR, bL, bR)
        gyro = RevIMU(hardwareMap)
        gyro.init()

        gamepadEx1 = GamepadEx(gamepad1)
        gamepadEx2 = GamepadEx(gamepad2)

        intakeToggle = Toggle(gamepadEx2, GamepadKeys.Button.A)
        driveToggle = TwoButtonToggle(gamepadEx1, GamepadKeys.Button.A, GamepadKeys.Button.B)
    }

    override fun loop() {
        if(driveToggle.state == TwoButtonToggle.State.A) {
            drive.driveFieldCentric(
                gamepadEx1.leftX,
                gamepadEx1.leftY,
                gamepadEx1.rightX,
                gyro.heading - gyroOffset
            )
        } else {
            drive.driveRobotCentric(
                gamepadEx1.leftX,
                gamepadEx1.leftY,
                gamepadEx1.rightX
            )
        }

        elevator.set(gamepadEx2.leftY)
        arm.set(gamepadEx2.rightX)
        intake.turnToAngle(if (intakeToggle.state) -45.0 else -130.0)

        telemetry.addData("gyro", gyro.heading)
        telemetry.addData("elevator", elevator.positions)
        telemetry.addData("arm", arm.currentPosition)
        telemetry.addData("intake", intake.angle)
        telemetry.addData("intakePos", intakeToggle.state)
        telemetry.update()
        intakeToggle.update()
    }
}