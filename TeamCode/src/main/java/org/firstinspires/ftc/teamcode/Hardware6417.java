package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    THIS IS THE HARDWARE CLASS FOR NEILBOT IN THE 2023 POWER PLAY SEASON

    This class is where the motors and servos are declared and called.
    All of the physical movements of the robot are in this class
 */

public class Hardware6417 {
    DcMotorEx frontLeft, frontRight, backLeft, backRight, slider, arm;
    Servo wrist,grabber;

    // declares motors
    public Hardware6417(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class,"front left");
        frontRight = hwMap.get(DcMotorEx.class, "front right");
        backLeft = hwMap.get(DcMotorEx.class, "back left");
        backRight = hwMap.get(DcMotorEx.class, "back right");

        slider = hwMap.get(DcMotorEx.class, "slider");
        arm = hwMap.get(DcMotorEx.class, "arm");

        wrist = hwMap.get(Servo.class, "wrist");
        grabber = hwMap.get(Servo.class,"grabber");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // resets encoders, sets runmode
    public void resetSlider() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setPower(0);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetArm() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void openGrabber() {
        if(grabber.getPosition() != Constants.grabberOpen) {
            grabber.setPosition(Constants.grabberOpen);
        }
    }

    public void closeGrabber() {
        if(grabber.getPosition() != Constants.grabberClose) {
            grabber.setPosition(Constants.grabberClose);
        }
    }

    public void autoSlider(int position) {
        // check if slider goes up or down and
        // set power accordingly
        if(slider.getCurrentPosition() > position) {
            slider.setPower(Constants.slideDownPower);
        } else if(slider.getCurrentPosition() < position) {
            slider.setPower(Constants.slideUpPower);
        }

        if(slider.getTargetPosition() != position) {
            slider.setTargetPosition(position);
        }
    }

    public void autoSlider(double power, int position) {
        slider.setPower(power);
        slider.setTargetPosition(position);
    }

    public void autoArm(double power, int position) {
        arm.setPower(power);
        if(arm.getCurrentPosition() != position && arm.getTargetPosition() != position) {
            arm.setTargetPosition(position);
        }
    }

    public void autoArm(int position, int dunk) {
        int target = position + dunk;
        if(dunk == 0) {
            arm.setPower(Constants.armFastPower);
        } else {
            arm.setPower(Constants.armSlowPower);
        }
        if(arm.getCurrentPosition() != target && arm.getTargetPosition() != target) {
            arm.setTargetPosition(target);
        }
    }

    public void autoWrist(double position) {
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    public boolean armNear(int target) {
        return Math.abs(arm.getCurrentPosition() - target) < Constants.armNearBack;
    }

    public boolean sliderAbove(int target) {
        return slider.getCurrentPosition() > target;
    }

    public void bobSlider() {
        slider.setPower(Constants.slideBobPower);
        slider.setTargetPosition(Constants.slideBobPos + 20);
    }

    public boolean bobDone() {
        return slider.getCurrentPosition() > Constants.slideBobPos;
    }

    public void telemetry(Telemetry tele) {
        tele.addData("slider position", slider.getCurrentPosition());
        tele.addData("slider power", slider.getPower());
        tele.addData("Arm position: ", arm.getCurrentPosition());
        tele.addData("Arm Power: ", arm.getPower());
        tele.addData("Grabber position: ", grabber.getPosition());
        tele.addData("Wrist position: ", wrist.getPosition());
    }

    // sets powers to drive motors
    public void clipJoyMecanumDrive(double vert, double horz, double rotate, double driveSpeed){
        double frDrive = (-vert + horz + rotate) * Constants.driveTuningFR;
        double flDrive = (-vert - horz - rotate) * Constants.driveTuningFL;
        double brDrive = (-vert - horz + rotate) * Constants.driveTuningBR;
        double blDrive = (-vert + horz - rotate) * Constants.driveTuningBL;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frDrive),Math.max(Math.abs(flDrive),Math.max(Math.abs(brDrive),Math.abs(blDrive)))));

        // power calculations
        if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1) {
            frontRight.setPower(driveSpeed * frDrive / max);
            frontLeft.setPower(driveSpeed * flDrive / max);
            backRight.setPower(driveSpeed * brDrive / max);
            backLeft.setPower(driveSpeed * blDrive / max);
        } else {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
        }
    }
}