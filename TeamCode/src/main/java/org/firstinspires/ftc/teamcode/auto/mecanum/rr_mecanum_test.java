package org.firstinspires.ftc.teamcode.auto.mecanum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="rr_mecanum_test", group="mecanum")
public class rr_mecanum_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory shootingpos = drivetrain.trajectoryBuilder(new Pose2d(0,0,0)).splineToConstantHeading(new Vector2d(65,-10),Math.toRadians(0)).build();

        //Trajectory forward65 = drivetrain.trajectoryBuilder(new Pose2d(0,0, 0)).forward(65).build();

        //Trajectory strafeleft = drivetrain.trajectoryBuilder(forward65.end()).strafeLeft(10).build();

        //drivetrain.followTrajectory(forward65);
        //drivetrain.followTrajectory(strafeleft);
        drivetrain.followTrajectory(shootingpos);
        sleep(3000);


    }
}
