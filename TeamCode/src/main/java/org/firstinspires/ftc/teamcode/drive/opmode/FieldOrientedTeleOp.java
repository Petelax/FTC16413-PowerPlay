package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@TeleOp
public class FieldOrientedTeleOp extends OpMode {

    private Motor fL, fR, bL, bR, elevator0, elevator1, arm, intake;
    private MecanumDrive drive;
    private GamepadEx gamepadEx1, gamepadEx2;
    private RevIMU gyro;
    private OpenCvCamera webcam;
    private MotorGroup elevator;
    //private PIDFController elevatorPIDF, armPIDF;

    int elevatorSP, armSP = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fL = new Motor(hardwareMap, "fL", 448, 375);
        fR = new Motor(hardwareMap, "fR", 448, 375);
        bL = new Motor(hardwareMap, "bL", 448, 375);
        bR = new Motor(hardwareMap, "bR", 448, 375);
        elevator0 = new Motor(hardwareMap, "elevator0", Motor.GoBILDA.RPM_435);
        elevator1 = new Motor(hardwareMap, "elevator1", Motor.GoBILDA.RPM_435);
        arm = new Motor(hardwareMap, "arm");
        intake = new Motor(hardwareMap, "intake");

        /*
        elevatorPIDF = new PIDFController(0.001, 0, 0, 0);
        armPIDF = new PIDFController(0.001, 0, 0, 0);
        */

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //intake.setInverted(true);
        intake.resetEncoder();

        elevator1.setInverted(true);
        elevator = new MotorGroup(elevator0, elevator1);
        //elevator.setRunMode(Motor.RunMode.PositionControl);
        elevator.resetEncoder();
        elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //elevator.setRunMode(Motor.RunMode.VelocityControl);
        //arm.setRunMode(Motor.RunMode.VelocityControl);


        drive = new MecanumDrive(fL, fR, bL, bR);
        gyro = new RevIMU(hardwareMap);
        gyro.init();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        drive.driveFieldCentric(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX(),
                gyro.getHeading());

        /*
        List<Double> positions = elevator.getPositions();
        double elevatorOutput = elevatorPIDF.calculate((positions.get(0) + positions.get(1))/2, elevatorSP);
        elevator.set(elevatorOutput);

        double armOutput = armPIDF.calculate(arm.getCurrentPosition(), armSP);
        arm.set(armOutput);

        elevatorSP += (int) (10 * gamepadEx2.getLeftY());
        armSP += (int) (10 * gamepadEx2.getRightX());
        */

        //elevator.setTargetPosition(elevatorDistance);
        double elevatorStick = gamepadEx2.getLeftY();
        // 38000-10
        if (elevator1.getCurrentPosition() > 29000 && elevatorStick > 0) {
            elevator.set(0);
        } else if (elevator1.getCurrentPosition() < 400 && elevatorStick < 0) {
            elevator.set(0);
        } else if ((elevatorStick < 0.01) && (elevatorStick > -0.01)) {
            elevator.stopMotor();
        } else if (elevatorStick < 0) {
            elevator.set(elevatorStick*0.5);
        } else {
            elevator.set(elevatorStick);
        }


        arm.set(gamepadEx2.getRightX());
        double intakeSpeed = 0;
        if(intake.getCurrentPosition() > 1150 && intake.getCurrentPosition() < 2600) {
            intakeSpeed = 0.75;
        }

        intakeSpeed += gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        intake.set(-1*intakeSpeed);

        telemetry.addData("gyro", gyro.getHeading());
        telemetry.addData("elevator", elevator.getPositions());
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("intake", intake.getCurrentPosition());
        telemetry.update();
    }
}