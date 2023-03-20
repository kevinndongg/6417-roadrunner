package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware6417;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.SignalDetectorPipeline;
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

@Autonomous(name = "Park Auto")
public class ParkAuto extends LinearOpMode {

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
        Trajectory backPush = drive.trajectoryBuilder(new Pose2d())
                .back(40)
                .build();

        Trajectory forward = drive.trajectoryBuilder(backPush.end())
                .forward(10)
                .build();

        Trajectory strafeR = drive.trajectoryBuilder(forward.end())
                .strafeLeft(33)
                .build();

        Trajectory strafeL = drive.trajectoryBuilder(forward.end())
                .strafeRight(33)
                .build();



        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.resetSlider();
        robot.resetArm();

        robot.closeGrabber();

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
                robot.autoWrist(Constants.wristUp);
                robot.autoArm(Constants.armSlowPower,Constants.autoArmUp);
                drive.followTrajectory(backPush);
                drive.followTrajectory(forward);
                drive.followTrajectory(strafeL);
                robot.autoArm(Constants.armSlowPower,0);
                robot.autoWrist(Constants.wristDown);
                break;
            case 1:
                robot.autoWrist(Constants.wristUp);
                robot.autoArm(Constants.armSlowPower,Constants.autoArmUp);
                drive.followTrajectory(backPush);
                drive.followTrajectory(forward);
                robot.autoArm(Constants.armSlowPower,0);
                robot.autoWrist(Constants.wristDown);
                break;
            case 2:
                robot.autoWrist(Constants.wristUp);
                robot.autoArm(Constants.armSlowPower, Constants.autoArmUp);
                drive.followTrajectory(backPush);
                drive.followTrajectory(forward);
                drive.followTrajectory(strafeR);
                robot.autoArm(Constants.armSlowPower,0);
                robot.autoWrist(Constants.wristDown);
                break;
        }



        while(opModeIsActive()){


            // telemetry for testing
            telemetry.addData("position: ", position);
            robot.telemetry(telemetry);
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
