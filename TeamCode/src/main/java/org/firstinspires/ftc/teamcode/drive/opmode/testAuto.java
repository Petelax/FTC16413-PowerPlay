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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import java.util.List;

@Autonomous
public class testAuto extends LinearOpMode {

    private Motor fL, fR, bL, bR, elevator0, elevator1, arm, intake;
    private MecanumDrive drive;
    private GamepadEx gamepadEx1, gamepadEx2;
    private RevIMU gyro;
    private OpenCvCamera webcam;
    private MotorGroup elevator;
    //private PIDFController elevatorPIDF, armPIDF;
    private int leftTarget;
    private int rightTarget;

    int elevatorSP, armSP = 0;

    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();



    }
    /*
    private void drive(double setpoint, DcMotor left, DcMotor right) {
        telemetry.addData("State", Integer.toString(error));
        telemetry.update();
        int target = (int)(setpoint * COUNTS_PER_INCH);

        leftTarget = leftTarget + target;
        rightTarget = rightTarget + target;

        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(speed);

        while(left.isBusy() && right.isBusy()) {
            telemetry.addData("Error", Integer.toString(error));
            telemetry.update();
        }
    }

    private void turn(double rotation, DcMotor left, DcMotor right) {
        int target = (int) ((rotation / 180)*Math.PI*RADIUS*COUNTS_PER_INCH);

        leftTarget = leftTarget + target;
        rightTarget = rightTarget - target;

        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(speed);
        right.setPower(-speed);

        while(left.isBusy() && right.isBusy()) {
            telemetry.addData("Error", target-((left.getCurrentPosition()+right.getCurrentPosition())/2));
            telemetry.update();
        }
    }

     */
}