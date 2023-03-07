package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    CONTROLS:
        One controller (gamepad 1):
            options + dpad down: SWITCH CONTROL TYPE

            RIGHTTRIGGER controls:
                left joystick: strafe
                right joystick: turn
                left trigger: slower driving (hold)

                left bumper: open/close grabber (toggle)
                right bumper: dunk arm

                right trigger: *HOLD FOR 2nd CONTROLS* wrist up, fast driving
                (release right trigger: intake position)

                *2nd* a (cross): maneuvering position
                *2nd* b (circle): outtake on low junction
                *2nd* x (square): outtake on high junction
                *2nd* y (triangle): outtake on medium junction

            SIMPLE controls:
                left joystick: strafe
                right joystick: turn
                left trigger: slower driving (hold)

                left bumper: open/close grabber (toggle)
                right bumper: dunk arm

                a (cross): intake position
                b (circle): outtake on low junction
                x (square): outtake on high junction
                y (triangle): outtake on medium junction
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

                dpad up: raise slider (hold)
                dpad down: lower slider (hold)
 */

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    double vert,horz,rotate;
    boolean slowDrive;
    boolean singleController;
    boolean lastLB1 = false;
    boolean lastOpDD1 = false;

    boolean grabbing = false;
    int armDunk;
    int manualSlideDelta = 0;

    enum SINGLECONTROL {
        SIMPLE, RIGHTTRIGGER
    }

    enum DOUBLECONTROL {
        SIMPLE
    }

    SINGLECONTROL controls1 = SINGLECONTROL.RIGHTTRIGGER;
    DOUBLECONTROL controls2 = DOUBLECONTROL.SIMPLE;


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

            // ONE CONTROLLER
            if(singleController) {
                // SWITCH CONTROL TYPE
                if (gamepad1.options && gamepad1.dpad_down && !lastOpDD1) {
                    if(controls1 == SINGLECONTROL.RIGHTTRIGGER) {
                        controls1 = SINGLECONTROL.SIMPLE;
                    }
                    else {
                        controls1 = SINGLECONTROL.RIGHTTRIGGER;
                    }
                }
                lastOpDD1 = gamepad1.options && gamepad1.dpad_down;

                switch(controls1) {
                    // RIGHTTRIGGER CONTROLS
                    case RIGHTTRIGGER:
                        // GRABBER CONTROL
                        if (gamepad1.left_bumper && !lastLB1) {
                            grabbing = !grabbing;
                        }
                        lastLB1 = gamepad1.left_bumper;

                        // slow drive
                        if (gamepad1.left_trigger > 0.1) {
                            slowDrive = true;
                        } else {
                            slowDrive = false;
                        }

                        // HOLD RIGHT TRIGGER CONTROLS
                        if (gamepad1.right_trigger > 0.1) {
                            if (states.getRobotState() == States6417.ROBOTSTATE.INTAKE) {
                                states.setRobotState(States6417.ROBOTSTATE.MANEUVERING);
                            }

                            if(gamepad1.a) {
                                states.setRobotState(States6417.ROBOTSTATE.MANEUVERING);
                            }

                            if (gamepad1.b) {
                                states.setRobotState(States6417.ROBOTSTATE.OUTTAKELOW);
                            }

                            if (gamepad1.y) {
                                states.setRobotState(States6417.ROBOTSTATE.OUTTAKEMED);
                            }

                            if (gamepad1.x) {
                                states.setRobotState(States6417.ROBOTSTATE.OUTTAKEHIGH);
                            }
                        } else {
                            states.setRobotState(States6417.ROBOTSTATE.INTAKE);
                        }

                        // dunk arm
                        if (gamepad1.right_bumper) {
                            armDunk = Constants.armDunk;
                        } else {
                            armDunk = 0;
                        }
                        break;
                    case SIMPLE:
                        // grab
                        if (gamepad1.left_bumper && !lastLB1) {
                            grabbing = !grabbing;
                        }
                        lastLB1 = gamepad1.left_bumper;

                        // slow drive
                        if (gamepad1.left_trigger > 0.1) {
                            slowDrive = true;
                        } else {
                            slowDrive = false;
                        }

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

                        // dunk arm
                        if (gamepad1.right_bumper) {
                            armDunk = Constants.armDunk;
                        } else {
                            armDunk = 0;
                        }
                        break;
                }
            // TWO CONTROLLER CONTROLS
            } else {
                // GRABBER CONTROLS
                if(gamepad1.left_bumper && !lastLB1){
                    grabbing = !grabbing;
                }
                lastLB1 = gamepad1.left_bumper;

                if (gamepad1.left_trigger > 0.1) {
                    slowDrive = true;
                } else {
                    slowDrive = false;
                }

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

                if(gamepad2.dpad_up) {
                    manualSlideDelta = 100;
                } else if(gamepad2.dpad_down) {
                    manualSlideDelta = -100;
                } else {
                    manualSlideDelta = 0;
                }

                // dunk arm
                if(gamepad2.left_trigger > 0.1) {
                    armDunk = Constants.armDunk;
                } else {
                    armDunk = 0;
                }
            }

            // moves the robot lol
            states.moveRobot(slowDrive, vert, horz, rotate, armDunk, manualSlideDelta);

            // telemetry
            telemetry.addData("singleController: ", singleController);
            telemetry.addData("controls1: ", controls1);
            telemetry.addData("controls2: ", controls2);
            states.telemetry(telemetry);
            telemetry.update();
        }
    }
}
