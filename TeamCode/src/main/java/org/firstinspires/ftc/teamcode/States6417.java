package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class States6417 {
    // High level enums
    public enum ROBOTSTATE {
        INTAKE, MANEUVERING, OUTTAKELOW, OUTTAKEMED, OUTTAKEHIGH
    }

    // Lower level enums

    enum SLIDESTATE {
        ZERO, MEDIUM, HIGH, BOBBING
    }

    enum ARMSTATE {
        GROUNDFRONT, OUTTAKELOW, OUTTAKEMED, OUTTAKEHIGH
    }

    enum WRISTSTATE {
        DOWN, UP
    }

    ElapsedTime sliderTimer;

    Hardware6417 robot;

    ROBOTSTATE robotState;
    ROBOTSTATE lastRobotState;
    SLIDESTATE slideState;
    ARMSTATE armState;
    WRISTSTATE wristState;
    double driveSpeed;

    public States6417(HardwareMap hwMap) {
        robot = new Hardware6417(hwMap);

        robotState = ROBOTSTATE.INTAKE;
        lastRobotState = ROBOTSTATE.INTAKE;
        slideState = SLIDESTATE.ZERO;
        armState = ARMSTATE.GROUNDFRONT;
        wristState = WRISTSTATE.DOWN;
        driveSpeed = 0;

        sliderTimer = new ElapsedTime();
    }

    public void moveRobot(boolean slowDrive, boolean fieldCentric, double vert, double horz, double rotate, int armDunk, int manualSliderDelta) {
        /*if(fieldCentric) {
            robot.clipFieldMecanumDrive(vert, horz, rotate, driveSpeed);
        } else {*/
            robot.clipBotMecanumDrive(vert, horz, rotate, driveSpeed);
        // }

        switch (robotState) {
            // INTAKE for when the robot is ready to pick up cones
            case INTAKE:
                // change slideState based on lastRobotState
                if (lastRobotState == ROBOTSTATE.MANEUVERING) {
                    slideState = SLIDESTATE.BOBBING;
                } else {
                    slideState = SLIDESTATE.ZERO;
                }

                // arm to the front
                armState = ARMSTATE.GROUNDFRONT;
                wristState = WRISTSTATE.DOWN;

                // set drive speeds
                if (slowDrive) {
                    driveSpeed = Constants.driveSpeedIntakeSlow;
                } else {
                    driveSpeed = Constants.driveSpeedIntake;
                }
                break;
            // MANEUVERING case for when robot has cone and is moving to score
            case MANEUVERING:
                // set slide to bottom
                if (slideState != SLIDESTATE.ZERO) {
                    slideState = SLIDESTATE.ZERO;
                }

                // set arm to front
                if (armState != ARMSTATE.GROUNDFRONT) {
                    armState = ARMSTATE.GROUNDFRONT;
                }

                if (wristState != WRISTSTATE.UP) {
                    wristState = WRISTSTATE.UP;
                }

                // drive
                if (slowDrive) {
                    driveSpeed = Constants.driveSpeedManeuveringSlow;
                } else {
                    driveSpeed = Constants.driveSpeedManeuvering;
                }
                break;
            case OUTTAKELOW:
                armState = ARMSTATE.OUTTAKELOW;

                wristState = WRISTSTATE.UP;

                slideState = SLIDESTATE.ZERO;

                // drive
                if(slowDrive) {
                    driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                } else {
                    driveSpeed = Constants.driveSpeedOuttakeUp;
                }
                break;
            case OUTTAKEMED:
                armState = ARMSTATE.OUTTAKEMED;
                slideState = SLIDESTATE.MEDIUM;
                wristState = WRISTSTATE.UP;

                // drive
                if(slowDrive) {
                    driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                } else {
                    driveSpeed = Constants.driveSpeedOuttakeUp;
                }
                break;
            case OUTTAKEHIGH:
                armState = ARMSTATE.OUTTAKEHIGH;
                slideState = SLIDESTATE.HIGH;
                wristState = WRISTSTATE.UP;

                // drive
                if(slowDrive) {
                    driveSpeed = Constants.driveSpeedOuttakeUpSlow;
                } else {
                    driveSpeed = Constants.driveSpeedOuttakeUp;
                }
                break;
        }

        switch (slideState) {
            // ZERO for slide bottom
            case ZERO:
                if(sliderTimer.seconds() > 2 && sliderTimer.seconds() < 4) {
                    robot.resetSlider();
                } else {
                    robot.autoSlider(0);
                }
                break;
            // MEDIUM for slide to medium junction preset
            case MEDIUM:
                robot.autoSlider(Constants.slideMediumPos + manualSliderDelta);
                break;
            case HIGH:
                robot.autoSlider(Constants.slideHighPos + manualSliderDelta);
                break;
            // BOBBING for when robot shifting from MANEUVERING to INTAKE
            case BOBBING:
                // makes slides go up
                robot.bobSlider();
                // once the slides reach position, change state to INTAKE
                if(robot.bobDone()) {
                    lastRobotState = robotState;
                    robotState = ROBOTSTATE.INTAKE;
                }
                break;
        }

        switch (armState) {

            // GROUNDFRONT for when robot is INTAKE and MANEUVERING
            case GROUNDFRONT:
                if(robot.sliderAbove(Constants.slideNearBottomPos)) {
                    robot.autoArm(Constants.armFastPower,50);
                } else {
                    robot.autoArm(Constants.armFastPower, 0);
                }
                break;

            case OUTTAKEHIGH:

            case OUTTAKEMED:
                robot.autoArm(Constants.armBackUpPos, armDunk);
                break;
            case OUTTAKELOW:
                robot.autoArm(Constants.armBackLowPos, armDunk);
                break;
        }

        switch (wristState) {
            case DOWN:
                robot.autoWrist(Constants.wristDown);
                break;
            case UP:
                robot.autoWrist(Constants.wristUp);
                break;
        }
    }

    public void resetSliderTimer() {
        sliderTimer.reset();
    }

    public void resetRobot() {
        robot.resetSlider();
        robot.resetArm();
        robot.openGrabber();
    }

    public void setRobotState(ROBOTSTATE robotState) {
        if(this.robotState != robotState) {
            this.lastRobotState = this.robotState;
            this.robotState = robotState;
        }
    }

    public ROBOTSTATE getRobotState() {
        return robotState;
    }

    public void openGrabber() {
        robot.openGrabber();
    }

    public void closeGrabber() {
        robot.closeGrabber();
    }

    public void telemetry(Telemetry tele) {
        tele.addData("lastRobotState", lastRobotState);
        tele.addData("robotState: ", robotState);
        tele.addData("slideState: ", slideState);
        tele.addData("armState: ", armState);
        robot.telemetry(tele);
    }
}
