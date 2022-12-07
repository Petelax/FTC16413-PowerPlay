package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class NoOdoAuto extends LinearOpMode {
    OpenCvCamera webcam;
    SignalPipeline signalPipeline;

    public static final double TICKS_PER_REV = 530.051282051282;

    public static final double MAX_RPM = 458.103;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
        Pose2d startPose = new Pose2d(-33, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(26)
                .build();

        Trajectory location1 = drive.trajectoryBuilder(forward.end())
                .strafeLeft(26)
                .build();

        Trajectory location3 = drive.trajectoryBuilder(forward.end())
                .strafeRight(26)
                .build();

        Trajectory location2 = drive.trajectoryBuilder(forward.end())
                .forward(25)
                .build();
        /*
        Trajectory location1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -34), Math.toRadians(90))
                .splineTo(new Vector2d(-12, -34), Math.toRadians(0))
                .build();

        Trajectory location2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -36), Math.toRadians(90))
                .build();

        Trajectory location3 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -36), Math.toRadians(90))
                .splineTo(new Vector2d(-46, -36), Math.toRadians(0))
                .build();

         */

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalPipeline = new SignalPipeline();
        webcam.setPipeline(signalPipeline);

        //webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("NOT SHEESH");

            }
        });

        /*
        int colour = signalPipeline.getColour();

        telemetry.addData("colour", colour);

         */

        waitForStart();

        if(isStopRequested()) return;

        /*
        while (opModeIsActive()) {
            int colour = signalPipeline.getColour();
            int colours[] = signalPipeline.getYCrCbVals();
            telemetry.addData("colour", colour);
            telemetry.addData("r", colours[0]);
            telemetry.addData("g", colours[1]);
            telemetry.addData("b", colours[2]);
            telemetry.update();
        }

         */

        int colour = signalPipeline.getColour();
        telemetry.addData("colour", colour);
        telemetry.update();

        forward(drive, 1000);
        sleep(1000);
        strafeLeft(drive, 1000);
        sleep(1000);
        strafeRight(drive, 1000);
        /*
        switch (colour) {
            case 1:
                telemetry.addLine("pos 1");
                break;
            case 2:
                telemetry.addLine("pos 2");
                break;
            case 3:
                telemetry.addLine("pos 3");
                break;
        }

         */
        telemetry.update();


        //drive.followTrajectory(location1);

        sleep(1000);

    }

    public void forward(SampleMecanumDrive drive, int mills) {
        drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
        sleep(mills);
        drive.setWeightedDrivePower(new Pose2d());
        sleep(50);
    }
    public void strafeLeft(SampleMecanumDrive drive, int mills) {
        drive.setWeightedDrivePower(new Pose2d(0, 0.2, 0));
        sleep(mills);
        drive.setWeightedDrivePower(new Pose2d());
        sleep(50);

    }
    public void strafeRight(SampleMecanumDrive drive, int mills) {
        drive.setWeightedDrivePower(new Pose2d(0, -0.2, 0));
        sleep(mills);
        drive.setWeightedDrivePower(new Pose2d());
        sleep(50);

    }
    public double distanceToTime(double dist, double speed) {
        double seconds = (MAX_RPM * speed * 60) / dist;

        return seconds;
    }

}