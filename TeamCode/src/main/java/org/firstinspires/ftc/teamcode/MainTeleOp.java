package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    //DistanceSensor distance;
    double driveSpeed;
    boolean singleController = true;

    // Enums for state machine

    // High level enums
    enum ROBOTSTATE {
        INTAKE, MANEUVERING, OUTTAKEGROUND, OUTTAKEUPHIGH,OUTTAKEUPLOW
    }

    // Lower level enums

    enum SLIDESTATE {
        ZERO, LOW, MEDIUM, HIGH,BOBBING
    }

    enum ARMSTATE {
        GROUNDFRONT,OUTTAKEBACKHIGH, OUTTAKEBACKLOW, GROUNDBACK
    }

    enum WRISTSTATE {
        DOWN, UP
    }

    public void runOpMode() throws InterruptedException
    {
        // hardware init
        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.resetMotors();

        // set states
        ROBOTSTATE robotState = ROBOTSTATE.INTAKE;
        ROBOTSTATE lastRobotState = ROBOTSTATE.INTAKE;

        SLIDESTATE slideState = SLIDESTATE.ZERO;

        ARMSTATE armState = ARMSTATE.GROUNDFRONT;

        WRISTSTATE wristState = WRISTSTATE.DOWN;

        // setup servos
        robot.openGrabber();

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

            // detect number of controllers connected
            if(gamepad2.getGamepadId() == -1) {
                singleController = true;
            } else {
                singleController = false;
            }

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // drive
            robot.clipJoyMecanumDrive(vert,horz,rotate,driveSpeed);


            // ROBOT STATE MACHINE


            switch (robotState) {
                // INTAKE for when the robot is ready to pick up cones
                case INTAKE:
                    // change slideState based on previous robotstate
                    // slides bob to make space for the wrist to go down correctly
                    if(lastRobotState == ROBOTSTATE.MANEUVERING) {
                        slideState = SLIDESTATE.BOBBING;
                    } else {
                        slideState = SLIDESTATE.ZERO;
                    }

                    // arm to the front
                    armState = ARMSTATE.GROUNDFRONT;

                    wristState = WRISTSTATE.DOWN;

                    // set drive speeds
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedIntakeSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedIntake;
                    }
                    break;

                // MANEUVERING case for when robot has cone and is moving to score
                case MANEUVERING:
                    // set slide to bottom
                    if(slideState != SLIDESTATE.ZERO) {
                        slideState = SLIDESTATE.ZERO;
                    }

                    // set arm to front
                    if(armState != ARMSTATE.GROUNDFRONT) {
                        armState = ARMSTATE.GROUNDFRONT;
                    }

                    if(wristState != WRISTSTATE.UP) {
                        wristState = WRISTSTATE.UP;
                    }

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedManeuveringSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedManeuvering;
                    }
                    break;

                // OUTTAKEGROUND for when dropping a cone to ground junction or terminal on ground
                /*case OUTTAKEGROUND:
                    // set arm to ground back position
                    if(armState != ARMSTATE.GROUNDBACK) {
                        armState = ARMSTATE.GROUNDBACK;
                    }

                    // set slides to 0
                    if(slideState != SLIDESTATE.ZERO) {
                        slideState = SLIDESTATE.ZERO;
                    }

                    if(wristState != WRISTSTATE.DOWN) {
                        wristState = WRISTSTATE.DOWN;
                    }

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeGroundSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeGround;
                    }
                    break;*/
                // OUTTAKEUP for dropping cones on med, high poles
                case OUTTAKEUPHIGH:
                    armState = ARMSTATE.OUTTAKEBACKHIGH;

                    wristState = WRISTSTATE.UP;

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeUp;
                    }
                    break;
                // OUTTAKEUPLOW for dropping cones on low poles
                case OUTTAKEUPLOW:
                    armState = ARMSTATE.OUTTAKEBACKLOW;

                    wristState = WRISTSTATE.UP;

                    // drive
                    if(gamepad1.left_trigger > 0.1) {
                        driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                    } else {
                        driveSpeed = Constants.driveSpeedOuttakeUp;
                    }
            }


            // SLIDE CONTROL


            switch (slideState) {
                // ZERO for slide bottom
                case ZERO:
                    robot.autoSlide(0);
                    break;
                // LOW for slide to low junction preset
                case LOW:
                    robot.autoSlide(Constants.slideLowPos);
                    break;

                // MEDIUM for slide to medium junction preset
                case MEDIUM:
                    robot.autoSlide(Constants.slideMediumPos);
                    break;
                case HIGH:
                    robot.autoSlide(Constants.slideHighPos);
                    break;
                // BOBBING for when robot shifting from MANEUVERING to INTAKE
                case BOBBING:
                    // makes slides go up
                    robot.bobSlide();
                    // once the slides reach position, change state to INTAKE
                    if(robot.bobDone()) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.INTAKE;
                    }
                    break;
            }


            // ARM CONTROL


            switch (armState) {

                // GROUNDFRONT for when robot is INTAKE and MANEUVERING
                case GROUNDFRONT:
                    robot.autoArm(Constants.armFastPower,0);
                    break;

                // MOVINGUP for when arm is going to OUTTAKEUP
                // arm should be fast until past top position
                case OUTTAKEBACKHIGH:
                    if(robot.armNear(Constants.armBackUpPos)) {
                        // if arm is past top, slow arm to back
                        robot.autoArm(Constants.armSlowPower, Constants.armBackUpPos);
                    } else {
                        // if arm is not past top, arm fast to back
                        robot.autoArm(Constants.armFastPower,Constants.armBackUpPos);
                    }
                    break;
                case OUTTAKEBACKLOW:
                    if(robot.armNear(Constants.armBackLowPos)) {
                        robot.autoArm(Constants.armSlowPower, Constants.armBackLowPos);
                    } else {
                        // if arm is not past top, arm fast to back
                        robot.autoArm(Constants.armFastPower, Constants.armBackLowPos);
                    }
                    break;
                /*case GROUNDBACK:
                    robot.autoArm(Constants.armFastPower, Constants.armGroundBackPos);
                    break;*/
            }


            // WRIST CONTROL


            switch (wristState) {
                case DOWN:
                    robot.autoWrist(Constants.wristDown);
                    break;
                case UP:
                    robot.autoWrist(Constants.wristUp);
                    break;
            }


            // GAMEPAD CONTROLS


            // ONE CONTROLLER CONTROLS
            if(singleController) {
                // grabber closed preset
                if (gamepad1.left_bumper) {
                    robot.closeGrabber();
                }

                // grabber open preset
                if (gamepad1.right_bumper) {
                    robot.openGrabber();
                }

                // sets state to maneuvering
                if (gamepad1.x && robotState != ROBOTSTATE.MANEUVERING) {
                    lastRobotState = robotState;
                    robotState = ROBOTSTATE.MANEUVERING;
                }

                // intake
                if (gamepad1.a && robotState != ROBOTSTATE.INTAKE) {
                    lastRobotState = robotState;
                    robotState = ROBOTSTATE.INTAKE;
                }

                // low preset
                if (gamepad1.b) {
                    if (robotState != ROBOTSTATE.OUTTAKEUPLOW) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPLOW;
                    }

                    if (slideState != SLIDESTATE.LOW) {
                        slideState = SLIDESTATE.LOW;
                    }
                }

                // medium preset
                if (gamepad1.y) {
                    if (robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                    }

                    if (slideState != SLIDESTATE.MEDIUM) {
                        slideState = SLIDESTATE.MEDIUM;
                    }
                }

                if(gamepad1.options) {
                    if(robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                    }

                    if(slideState != SLIDESTATE.HIGH) {
                        slideState = SLIDESTATE.HIGH;
                    }
                }
                // sets state to outtakeground
                /*if(gamepad1.right_trigger > 0.1) {
                    if(robotState != ROBOTSTATE.OUTTAKEGROUND) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEGROUND;
                    }
                }*/

            // TWO CONTROLLER CONTROLS
            } else {
                if (gamepad1.left_bumper) {
                    robot.closeGrabber();
                }

                // grabber open preset
                if (gamepad1.right_bumper) {
                    robot.openGrabber();
                }

                // sets state to maneuvering
                if (gamepad1.a && robotState != ROBOTSTATE.MANEUVERING) {
                    lastRobotState = robotState;
                    robotState = ROBOTSTATE.MANEUVERING;
                }

                // intake
                if (gamepad2.a && robotState != ROBOTSTATE.INTAKE) {
                    lastRobotState = robotState;
                    robotState = ROBOTSTATE.INTAKE;
                }

                // low preset
                if (gamepad2.b) {
                    if (robotState != ROBOTSTATE.OUTTAKEUPLOW) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPLOW;
                    }

                    if (slideState != SLIDESTATE.LOW) {
                        slideState = SLIDESTATE.LOW;
                    }
                }

                // medium preset
                if (gamepad2.y) {
                    if (robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                    }

                    if (slideState != SLIDESTATE.MEDIUM) {
                        slideState = SLIDESTATE.MEDIUM;
                    }
                }

                if(gamepad2.x) {
                    if(robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                    }

                    if(slideState != SLIDESTATE.HIGH) {
                        slideState = SLIDESTATE.HIGH;
                    }
                }
            }

            // telemetry for testing
            telemetry.addData("singleController", singleController);
            telemetry.addData("lastRobotState", lastRobotState);
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
