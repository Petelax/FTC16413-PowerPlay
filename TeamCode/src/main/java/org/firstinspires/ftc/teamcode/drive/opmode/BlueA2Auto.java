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
public class BlueA2Auto extends LinearOpMode {
    OpenCvCamera webcam;
    SignalPipeline signalPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(33, -60, Math.toRadians(90));

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

        telemetry.addData("colour", signalPipeline.getColour());

        drive.followTrajectory(forward);
        switch (signalPipeline.getColour()) {
            case 1:
                drive.followTrajectory(location1);
                break;
            case 2:
                drive.followTrajectory(location2);
                break;
            case 3:
                drive.followTrajectory(location3);
                break;
        }



        PoseStorage.currentPose = drive.getPoseEstimate();
    }

}