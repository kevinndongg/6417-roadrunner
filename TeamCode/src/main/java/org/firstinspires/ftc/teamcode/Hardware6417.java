package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
    HARDWARE CLASS FOR NEILBOT IN THE 2023 POWER PLAY SEASON

    This class is where the motors and servos are declared and called.
    All of the physical movements of the robot are in this class
 */

public class Hardware6417 {
    DcMotorEx frontLeft, frontRight, backLeft, backRight, slider, arm;
    Servo wrist,grabber;

    BNO055IMU imu;

    // constructor
    public Hardware6417(HardwareMap hwMap) {
        initDrive(hwMap);
        initIntake(hwMap);
    }

    public void initDrive(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class,"front left");
        frontRight = hwMap.get(DcMotorEx.class, "front right");
        backLeft = hwMap.get(DcMotorEx.class, "back left");
        backRight = hwMap.get(DcMotorEx.class, "back right");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initIntake(HardwareMap hwMap) {
        slider = hwMap.get(DcMotorEx.class, "slider");
        arm = hwMap.get(DcMotorEx.class, "arm");

        wrist = hwMap.get(Servo.class, "wrist");
        grabber = hwMap.get(Servo.class,"grabber");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initImu(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
    }

    // RESET METHODS
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

    // GRABBER METHODS

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

    // ARM METHODS

    public void autoArm(double power, int position) {
        arm.setPower(power);
        if(arm.getCurrentPosition() != position && arm.getTargetPosition() != position) {
            arm.setTargetPosition(position);
        }
    }

    public void autoArm(int position, int dunk) {
        int target = position + dunk;
        if(dunk == 0 || arm.getCurrentPosition() < position) {
            arm.setPower(Constants.armFastPower);
        } else {
            arm.setPower(Constants.armSlowPower);
        }
        if(arm.getCurrentPosition() != target && arm.getTargetPosition() != target) {
            arm.setTargetPosition(target);
        }
    }

    // WRIST METHODS

    public void autoWrist(double position) {
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    // SLIDER METHODS

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

    // IMU METHODS

    public double getReferenceAngle(){
        return (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + Math.PI/2) % (Math.PI*2);
    }

    // DRIVE METHODS

    // sets powers to drive motors
    public void clipBotMecanumDrive(double vert, double horz, double rotate, double driveSpeed){
        double frDrive = (vert + horz + rotate) * Constants.driveTuningFR;
        double flDrive = (vert - horz - rotate) * Constants.driveTuningFL;
        double brDrive = (vert - horz + rotate) * Constants.driveTuningBR;
        double blDrive = (vert + horz - rotate) * Constants.driveTuningBL;

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

    public void clipFieldMecanumDrive(double vert, double horz, double rotate, double driveSpeed, double driveAngle) {
        // finding driveMagnitude of drive vector / rotate
        double driveMagnitude = Math.max(Math.abs(vert), Math.max(Math.abs(horz), Math.abs(rotate)));
        double vertHorzMagnitude = Math.max(Math.abs(vert), Math.abs(horz));
        double vertControl = Math.sin(driveAngle) * vertHorzMagnitude;
        double horzControl = Math.cos(driveAngle) * vertHorzMagnitude;

        double frDrive = (vertControl - horzControl - rotate) * Constants.driveTuningFR;
        double flDrive = (vertControl + horzControl + rotate) * Constants.driveTuningFL;
        double brDrive = (vertControl + horzControl - rotate) * Constants.driveTuningBR;
        double blDrive = (vertControl - horzControl + rotate) * Constants.driveTuningBL;

        double max = Math.abs(Math.max(Math.abs(frDrive),Math.max(Math.abs(flDrive),Math.max(Math.abs(brDrive),Math.abs(blDrive)))));

        // power calculations
        if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1) {
            frontRight.setPower(driveSpeed * driveMagnitude * frDrive / max);
            frontLeft.setPower(driveSpeed * driveMagnitude * flDrive / max);
            backRight.setPower(driveSpeed * driveMagnitude * brDrive / max);
            backLeft.setPower(driveSpeed * driveMagnitude * blDrive / max);
        } else {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
        }
    }

    public void telemetry(Telemetry tele) {
        tele.addData("slider position", slider.getCurrentPosition());
        tele.addData("slider power", slider.getPower());
        tele.addData("Arm position: ", arm.getCurrentPosition());
        tele.addData("Arm Power: ", arm.getPower());
        tele.addData("Grabber position: ", grabber.getPosition());
        tele.addData("Wrist position: ", wrist.getPosition());
    }
}