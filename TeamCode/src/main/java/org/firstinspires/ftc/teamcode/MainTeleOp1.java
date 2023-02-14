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
    //DistanceSensor distance;
    double driveSpeed;

    // Enums for state machine

    // High level enums
    enum ROBOTSTATE {
        INTAKE, MANEUVERING, OUTTAKEGROUND, OUTTAKEUP
    }

    // Lower level enums

    enum SLIDESTATE {
        ZERO, LOW, MEDIUM
    }

    enum ARMSTATE {
        GROUNDFRONT, MOVINGUP,OUTTAKEBACK, GROUNDBACK
    }

    public void runOpMode() throws InterruptedException
    {
        // hardware init
        Hardware6417 robot = new Hardware6417(hardwareMap);

        // set states
        ROBOTSTATE robotState = ROBOTSTATE.INTAKE;
        ROBOTSTATE lastRobotState = ROBOTSTATE.INTAKE;

        SLIDESTATE slideState = SLIDESTATE.ZERO;
        SLIDESTATE lastSlideState = SLIDESTATE.ZERO;

        ARMSTATE armState = ARMSTATE.GROUNDFRONT;
        ARMSTATE lastArmState = ARMSTATE.GROUNDFRONT;

        // setup servos
        robot.openGrabber();
        robot.closeGrabber();

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

            // drive
            robot.clipJoyMecanumDrive(vert,horz,rotate,driveSpeed);


            switch (robotState) {
                case INTAKE:
                    lastSlideState = slideState;
                    slideState = SLIDESTATE.ZERO;
                    lastArmState = armState;
                    armState = ARMSTATE.GROUNDFRONT;

                    // set drive speeds
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedIntakeSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedIntake;
                    }
                    break;
                case MANEUVERING:
                    lastSlideState = slideState;
                    slideState = SLIDESTATE.ZERO;
                    lastArmState = armState;
                    armState = ARMSTATE.GROUNDFRONT;

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedManeuveringSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedManeuvering;
                    }
                    break;
                case OUTTAKEGROUND:
                    lastArmState = armState;
                    armState = ARMSTATE.GROUNDBACK;
                    lastSlideState = slideState;
                    slideState = SLIDESTATE.ZERO;

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeGroundSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeGround;
                    }
                    break;
                case OUTTAKEUP:
                    lastArmState = armState;
                    armState = ARMSTATE.OUTTAKEBACK;

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeUp;
                    }
                    break;
            }

            switch (slideState) {
                case ZERO:
                    robot.autoSlide(0);
                    break;
                case LOW:
                    robot.autoSlide(Constants.slideLow);
                    break;
                case MEDIUM:
                    robot.autoSlide(Constants.slideMedium);
                    break;
            }

            switch (armState) {
                case GROUNDFRONT:
                    robot.autoArm(Constants.armFastPower,0);
                    break;
                case MOVINGUP:
                    if(robot.armPastTop()) {
                        robot.autoArm(Constants.armSlowPower,Constants.armBack);
                    } else {
                        lastArmState = armState;
                        armState = ARMSTATE.OUTTAKEBACK;
                    }
                    break;
                case OUTTAKEBACK:
                    robot.autoArm(Constants.armSlowPower, Constants.armBack);
                    break;
            }

            // grabber closed preset
            if(gamepad1.left_bumper){
                robot.closeGrabber();
            }

            // grabber open preset
            if(gamepad1.right_bumper){
                robot.openGrabber();
            }

            // sets state to maneuvering
            if(gamepad1.x) {
                robot.wristUp();
                lastRobotState = robotState;
                robotState = ROBOTSTATE.MANEUVERING;
            }

            // intake
            if(gamepad1.a) {
                robot.wristDown();
                lastRobotState = robotState;
                robotState = ROBOTSTATE.INTAKE;
            }

            // low preset
            if(gamepad1.b) {
                lastRobotState = robotState;
                robotState = ROBOTSTATE.OUTTAKEUP;

                lastSlideState = slideState;
                slideState = SLIDESTATE.LOW;
            }

            // medium preset
            if(gamepad1.y) {
                lastRobotState = robotState;
                robotState = ROBOTSTATE.OUTTAKEUP;

                lastSlideState = slideState;
                slideState = SLIDESTATE.MEDIUM;
            }

            /*// while arm is going up and is past -750
            if(armGoingUp && arm.getCurrentPosition() > Constants.armTop) {
                // arm is slow
                arm.setPower(Constants.armSlowPower);
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
*/
            // telemetry for testing

            telemetry.addData("robotState: ", robotState);
            telemetry.addData("slideState: ", slideState);
            telemetry.addData("armState: ", armState);

            robot.telemetry(telemetry);
            // telemetry.addData("armGoingUp: ", armGoingUp);
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
