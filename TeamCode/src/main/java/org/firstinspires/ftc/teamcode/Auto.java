package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight, slider, arm;
    Servo grabber;
    Servo wrist;

    int position;

    public void runOpMode() throws InterruptedException
    {
        //camera declaration
        OpenCvCamera webcam;
        SignalDetectorPipeline pipeline;

        //initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SignalDetectorPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        }); //done initializing camera

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory forwardPush = drive.trajectoryBuilder(new Pose2d())
                .forward(37)
                .build();

        Trajectory back = drive.trajectoryBuilder(forwardPush.end())
                .back(10)
                .build();

        Trajectory strafeR = drive.trajectoryBuilder(back.end())
                .strafeLeft(23)
                .build();

        Trajectory strafeL = drive.trajectoryBuilder(back.end())
                .strafeRight(23)
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
            position = pipeline.position;
            telemetry.addData("position: ", position);
            telemetry.addData("red total: ", pipeline.redTotal);
            telemetry.addData("blue total: ", pipeline.blueTotal);
            telemetry.addData("green total: ", pipeline.greenTotal);
            telemetry.update();
        }

        waitForStart();
        resetRuntime();


        position = pipeline.position;
        webcam.stopStreaming();

        drive.setPoseEstimate(new Pose2d());


        switch(position) {
            case 0:
                grabber.setPosition(Constants.grabberClose);
                wrist.setPosition(Constants.wristUp);
                arm.setPower(0.25);
                arm.setTargetPosition(-900);
                drive.followTrajectory(forwardPush);
                drive.followTrajectory(back);
                drive.followTrajectory(strafeL);
                arm.setTargetPosition(0);
                wrist.setPosition(Constants.wristDown);
                break;
            case 1:
                grabber.setPosition(Constants.grabberClose);
                wrist.setPosition(Constants.wristUp);
                drive.followTrajectory(forwardPush);
                drive.followTrajectory(back);
                wrist.setPosition(Constants.wristDown);
                break;
            case 2:
                grabber.setPosition(Constants.grabberClose);
                wrist.setPosition(Constants.wristUp);
                arm.setPower(0.25);
                arm.setTargetPosition(-900);
                drive.followTrajectory(forwardPush);
                drive.followTrajectory(back);
                drive.followTrajectory(strafeR);
                arm.setTargetPosition(0);
                wrist.setPosition(Constants.wristDown);
                break;
        }



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
