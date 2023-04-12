package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "Distance Test", group = "TeleOp")
public class DistanceTest extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    BNO055IMU imu;
    DistanceSensor distance;

    double vert,horz,rotate;
    double cumulativeAngle, driveAngle, leftStickAngle;
    double driveSpeed;
    double angleOffset = Math.PI/2;

    boolean lastLB1 = false;

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        frontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back left");
        backRight = hardwareMap.get(DcMotorEx.class, "back right");

        distance = hardwareMap.get(DistanceSensor.class, "distance");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);


        // servo declarations

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            // drive calculations
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;


            cumulativeAngle = (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + Math.PI/2) % (Math.PI*2);

            if(gamepad1.left_bumper && !lastLB1) {
                angleOffset = cumulativeAngle;
            }
            lastLB1 = gamepad1.left_bumper;

            if(gamepad1.left_trigger > 0.1) {
                driveSpeed = 0.4;
            } else if(gamepad1.right_trigger > 0.1) {
                driveSpeed = 0.8;
            } else {
                driveSpeed = 0.6;
            }

            if(horz < 0) {
                leftStickAngle = Math.atan(vert / horz) + Math.PI;
            } else if(vert < 0) {
                leftStickAngle = Math.atan(vert / horz) + Math.PI * 2;
            } else if(vert == 0 && horz == 0) {
                leftStickAngle = 0;
            } else {
                leftStickAngle = Math.atan(vert / horz);
            }

            Drive(vert, horz, rotate);
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }

    public void Drive(double vert, double horz, double rotate){
        double magnitude;


        double driveAngle = (leftStickAngle - cumulativeAngle + angleOffset) % (Math.PI * 2);

        magnitude = Math.max(vert, Math.max(horz, rotate));

        double vertControl = magnitude*Math.sin(driveAngle);
        double horzControl = magnitude*Math.cos(driveAngle);

        double frDrive = vertControl - horzControl - rotate;
        double flDrive = vertControl + horzControl + rotate;
        double brDrive = vertControl + horzControl - rotate;
        double blDrive = vertControl - horzControl + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frDrive),Math.max(Math.abs(flDrive),Math.max(Math.abs(brDrive),Math.abs(blDrive)))));

        // power calculations
        frontRight.setPower(magnitude * driveSpeed * Constants.driveTuningFR * frDrive / max);
        frontLeft.setPower(magnitude * driveSpeed * Constants.driveTuningFL * flDrive / max);
        backRight.setPower(magnitude * driveSpeed * Constants.driveTuningBR * brDrive / max);
        backLeft.setPower(magnitude * driveSpeed * Constants.driveTuningBL * blDrive / max);
    }
}