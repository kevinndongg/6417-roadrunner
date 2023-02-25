package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/*
Questions for Andrew:
-lastDU2 & other booleans
-trajectorySequences
-headings, angle of robot
    -field centric
    -auto align
-using setWeightedDrivePower vs setting powers directly
-extending SampleMecanumDrive
 */

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight, slider, arm;
    Servo grabber;
    Servo wrist;

    int position;

    public void runOpMode() throws InterruptedException
    {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .splineTo(new Vector2d(10,0),Math.toRadians(90))
                .build();


        // motor declarations
        FrontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        FrontRight = hardwareMap.get(DcMotorEx.class, "front right");
        BackLeft = hardwareMap.get(DcMotorEx.class, "back left");
        BackRight = hardwareMap.get(DcMotorEx.class, "back right");
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0);

        while(opModeInInit()) {
        }

        waitForStart();
        resetRuntime();

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        drive.followTrajectorySequence(test);

        while(opModeIsActive()){


            // telemetry for testing
            telemetry.addData("position: ", position);
            telemetry.addData("slider position", slider.getCurrentPosition());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
