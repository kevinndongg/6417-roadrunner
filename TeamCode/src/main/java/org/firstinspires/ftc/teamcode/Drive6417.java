package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drive6417 {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public Drive6417(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class,"front left");
        frontRight = hwMap.get(DcMotorEx.class, "front right");
        backLeft = hwMap.get(DcMotorEx.class, "back left");
        backRight = hwMap.get(DcMotorEx.class, "back right");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPowers(double vert, double horz, double rotate, double driveSpeed){
        double frdrive = (-vert - horz - rotate) * Constants.driveTuningFR;
        double fldrive = (-vert + horz + rotate) * Constants.driveTuningFL;
        double brdrive = (-vert + horz - rotate) * Constants.driveTuningBR;
        double bldrive = (-vert - horz + rotate) * Constants.driveTuningBL;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        frontRight.setPower(driveSpeed  * frdrive / max);
        frontLeft.setPower(driveSpeed * fldrive / max);
        backRight.setPower(driveSpeed * brdrive / max);
        backLeft.setPower(driveSpeed * bldrive / max);
    }
}
