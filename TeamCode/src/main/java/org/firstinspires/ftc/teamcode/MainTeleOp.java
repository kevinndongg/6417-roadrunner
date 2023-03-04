package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    CONTROLS:
        One controller (gamepad 1):
            left joystick: strafe
            right joystick: turn
            left trigger: slower driving (hold)

            left bumper: open/close grabber (toggle)
            right bumper: dunk arm

            right trigger: wrist up, fast driving + enables outtake via *2ND CONTROLS*
            (release right trigger: intake position)
            *2nd* b (circle): outtake on low junction
            *2nd* x (square): outtake on high junction
            *2nd* y (triangle): outtake on medium junction

        Two Controllers:
            gamepad 1:
                left joystick: strafe
                right joystick: turn
                right trigger: slower driving (hold)

                left bumper: open/close grabber (toggle)

                b (circle): wrist up, fast driving
                a (cross): intake position

            gamepad 2:
                a (cross): intake position
                b (circle): outtake on low junction
                x (square): outtake on high junction
                y (triangle): outtake on medium junction

                left trigger: dunk arm
                right bumper: reset slider
 */

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    double vert,horz,rotate;
    boolean slowDrive;
    boolean singleController;
    boolean lastLB1 = false;
    boolean grabbing = false;
    int armDunk;



    public void runOpMode() throws InterruptedException
    {
        States6417 states = new States6417(hardwareMap);
        states.resetRobot();

        waitForStart();
        resetRuntime();

        while(opModeIsActive()){
            // detect number of controllers connected
            if(gamepad2.getGamepadId() == -1) {
                singleController = true;
            } else {
                singleController = false;
            }

            // drive calculations
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            // grabber control
            if(grabbing){
                states.closeGrabber();
            }
            else{
                states.openGrabber();
            }

            // CONTROLS

            // ONE CONTROLLER CONTROLS
            if(singleController) {
                // GRABBER CONTROL
                if(gamepad1.left_bumper && !lastLB1){
                    grabbing = !grabbing;
                }
                lastLB1 = gamepad1.left_bumper;

                // HOLD RIGHT TRIGGER CONTROLS
                /*if(gamepad1.right_trigger > 0.1) {
                    if(robotState == ROBOTSTATE.INTAKE) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.MANEUVERING;
                    }

                    if(gamepad1.b && robotState != ROBOTSTATE.OUTTAKEUPLOW) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.OUTTAKEUPLOW;
                    }

                    if(gamepad1.y) {
                        if(robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                            lastRobotState = robotState;
                            robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                        }

                        slideState = SLIDESTATE.MEDIUM;
                    }

                    if(gamepad1.x) {
                        if(robotState != ROBOTSTATE.OUTTAKEUPHIGH) {
                            lastRobotState = robotState;
                            robotState = ROBOTSTATE.OUTTAKEUPHIGH;
                        }

                        slideState = SLIDESTATE.HIGH;
                    }
                } else {
                    if(robotState != ROBOTSTATE.INTAKE) {
                        lastRobotState = robotState;
                        robotState = ROBOTSTATE.INTAKE;
                    }
                }*/

                // intake
                if (gamepad1.a) {
                    states.setRobotState(States6417.ROBOTSTATE.INTAKE);
                }

                // low preset
                if (gamepad1.b) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKELOW);
                }

                // medium preset
                if (gamepad1.y) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKEMED);
                }

                // high preset
                if(gamepad1.x) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKEHIGH);
                }

                if(gamepad1.right_trigger > 0.1) {
                    armDunk = Constants.armDunk;
                } else {
                    armDunk = 0;
                }

            // TWO CONTROLLER CONTROLS
            } else {
                // GRABBER CONTROLS
                if(gamepad1.left_bumper && !lastLB1){
                    grabbing = !grabbing;
                }
                lastLB1 = gamepad1.left_bumper;

                if (gamepad1.a) {
                    states.setRobotState(States6417.ROBOTSTATE.INTAKE);
                }

                // sets state to maneuvering
                if (gamepad1.b) {
                    states.setRobotState(States6417.ROBOTSTATE.MANEUVERING);
                }

                // intake
                if (gamepad2.a) {
                    states.setRobotState(States6417.ROBOTSTATE.INTAKE);
                }

                // low preset
                if (gamepad2.b) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKELOW);
                }

                // medium preset
                if (gamepad2.y) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKEMED);
                }

                // high preset
                if(gamepad2.x) {
                    states.setRobotState(States6417.ROBOTSTATE.OUTTAKEHIGH);
                }

                // dunk arm
                if(gamepad2.left_trigger > 0.1) {
                    armDunk = Constants.armDunk;
                } else {
                    armDunk = 0;
                }
            }

            // moves the robot lol
            states.moveRobot(slowDrive, vert, horz, rotate, armDunk);

            // telemetry for testing
            telemetry.addData("singleController", singleController);
            states.telemetry(telemetry);
            telemetry.update();
        }
    }
}
