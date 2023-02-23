package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/*
Questions for Andrew:
-lastDU2 & other booleans
-trajectorySequences
    -coordinates, field overlay ?
    -temporalMarkerOffset ?
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

        Pose2d startPos = new Pose2d(0,0,Math.toRadians(90));
        double startHeading = Math.toRadians(90);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(5,5),Math.toRadians(45))
                .build();

        TrajectorySequence test1 = drive.trajectorySequenceBuilder(startPos)
            .splineTo(new Vector2d(-5,-5),Math.toRadians(135))
            .build();

        Hardware6417 robot = new Hardware6417(hardwareMap);

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

        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));

        drive.followTrajectorySequence(test1);

        /*switch(position) {
            case 0:
                grabber.setPosition(Constants.grabberClose);
                wrist.setPosition(Constants.wristUp);
                arm.setPower(0.25);
                arm.setTargetPosition(-900);
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
        }*/



        while(opModeIsActive()){


            // telemetry for testing
            robot.telemetry(telemetry);
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
