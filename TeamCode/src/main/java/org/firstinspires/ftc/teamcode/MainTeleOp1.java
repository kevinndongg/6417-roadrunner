package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Main TeleOp 1", group = "TeleOp")
public class MainTeleOp1 extends LinearOpMode {
    DcMotorEx slider, arm;
    Servo grabber, wrist;
    //DistanceSensor distance;
    double driveSpeed;
    boolean armGoingUp = false;

    // Enums for state machine

    // High level enums
    enum ROBOTSTATE {
        INTAKE, MANEUVERING, OUTTAKEGROUND, OUTTAKEUP
    }

    // Lower level enums

    enum SLIDESTATE {
        ZERO, LOW, MEDIUM, HIGH
    }

    enum ARMSTATE {
        GROUNDFRONT, MANEUVERING, OUTTAKEBACK, GROUNDBACK
    }

    public void runOpMode() throws InterruptedException
    {
        // set states
        ROBOTSTATE robotState = ROBOTSTATE.INTAKE;
        ROBOTSTATE lastRobotState = ROBOTSTATE.INTAKE;

        SLIDESTATE slideState = SLIDESTATE.ZERO;
        SLIDESTATE lastSlideState = SLIDESTATE.ZERO;

        ARMSTATE armState = ARMSTATE.GROUNDFRONT;
        ARMSTATE lastArmState = ARMSTATE.GROUNDFRONT;

        // setup servos
        grabber.setPosition(Constants.grabberOpen);
        wrist.setPosition(Constants.wristDown);

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

        // hardware init
        Hardware6417 robot = new Hardware6417(hardwareMap);

        // motor declarations

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
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0);

        waitForStart();
        webcam.stopStreaming();
        resetRuntime();

        while(opModeIsActive()){
            // safety for switching controllers
            if(gamepad2.start || gamepad1.start){
                continue;
            }

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            /*if(gamepad1.left_trigger > 0.15) {
                driveSpeed = Constants.driveSpeedSlow + (1.0-gamepad1.left_trigger) * (Constants.driveSpeedNormal-Constants.driveSpeedSlow);
            } else  if(gamepad1.right_trigger > 0.1){
                driveSpeed = Constants.driveSpeedNormal + gamepad1.right_trigger * (Constants.driveSpeedFast - Constants.driveSpeedNormal);
            } else {
                driveSpeed = Constants.driveSpeedNormal;
            }*/

            // drive
            robot.clipJoyMecanumDrive(vert,horz,rotate,driveSpeed);

            switch (robotState) {
                case INTAKE:
                    // set drive speeds
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedIntakeSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedIntake;
                    }

                    if(gamepad1.x) {
                        wrist.setPosition(Constants.wristUp);
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.MANEUVERING;
                    }

                    if(gamepad1.b) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUP;

                        lastSlideState = slideState;
                        slideState = SLIDESTATE.LOW;

                        lastArmState = armState;
                        armState = ARMSTATE.OUTTAKEBACK;
                    }

                    if(gamepad1.y) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUP;

                        lastSlideState = slideState;
                        slideState = SLIDESTATE.MEDIUM;

                        lastArmState = armState;
                        armState = ARMSTATE.OUTTAKEBACK;
                    }
                    break;
                case MANEUVERING:
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedManeuveringSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedManeuvering;
                    }
                    break;
                case OUTTAKEGROUND:
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeGroundSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeGround;
                    }
                    break;
                case OUTTAKEUP:
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeUp;
                    }

            }

            // grabber closed preset
            if(gamepad1.left_bumper){
                grabber.setPosition(Constants.grabberClose);
            }

            // grabber open preset
            if(gamepad1.right_bumper){
                grabber.setPosition(Constants.grabberOpen);
            }

            if(gamepad1.dpad_up) {
                wrist.setPosition(Constants.wristUp);
            }

            if(gamepad1.dpad_down) {
                wrist.setPosition(Constants.wristDown);
            }

            // while arm is going up and is past -750
            if(armGoingUp && arm.getCurrentPosition() > Constants.armTop) {
                // arm is slow
                arm.setPower(Constants.armSlowSpeed);
                armGoingUp = false;
            }

            // Everything down
            if(gamepad1.a) {
                armGoingUp = false;
                slider.setPower(0.9);
                slider.setTargetPosition(0);
                arm.setPower(0.6);
                arm.setTargetPosition(0);
                wrist.setPosition(Constants.wristDown);
            }

            // arm back + slider up to low
            if(gamepad1.b) {
                armGoingUp = true;
                if(slider.getCurrentPosition() > Constants.slideLow){
                    slider.setPower(0.6);
                } else {
                    slider.setPower(0.4);
                }
                slider.setTargetPosition(Constants.slideLow);
                arm.setPower(0.7);
                arm.setTargetPosition(Constants.armBack);
                wrist.setPosition(Constants.wristUp);
            }

            if(gamepad1.y) {
                armGoingUp = true;
                slider.setPower(0.6);
                slider.setTargetPosition(Constants.slideMedium);
                arm.setPower(0.6);
                arm.setTargetPosition(Constants.armBack);
                wrist.setPosition(Constants.wristUp);
            }

            // telemetry for testing
            telemetry.addData("slider position", slider.getCurrentPosition());
            telemetry.addData("slider power", slider.getPower());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Arm Power: ", arm.getPower());
            telemetry.addData("robotState: ", robotState);
            telemetry.addData("slideState: ", slideState);
            telemetry.addData("armState: ", armState);
            // telemetry.addData("armGoingUp: ", armGoingUp);
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
