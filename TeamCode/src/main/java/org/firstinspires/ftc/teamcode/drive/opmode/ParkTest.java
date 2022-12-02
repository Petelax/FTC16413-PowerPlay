package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ParkTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -60, Math.toDegrees(90));

        Trajectory trajectory0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-24,-60), Math.toDegrees(90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory0);

        PoseStorage.currentPose = drive.getPoseEstimate();

    }

}
