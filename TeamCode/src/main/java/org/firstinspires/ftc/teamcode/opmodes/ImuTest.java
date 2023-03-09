package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ImuTest", group = "TeleOp")
public class ImuTest extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    BNO055IMU imu;

    double[] angles = new double[3];

    Orientation orientation = imu.getAngularOrientation();
    double driveSpeed = 0.4;

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        frontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back left");
        backRight = hardwareMap.get(DcMotorEx.class, "back right");

        // imu declaration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        // servo declarations

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        while(opModeIsActive()){
            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            Drive(vert, horz, rotate);


            getHeading(angles, imu, AngleUnit.DEGREES);

            telemetry.addData("angular orientation:", imu.getAngularOrientation());
            telemetry.addData("angle 1:", angles[0]);
            telemetry.addData("angle 2:", angles[1]);
            telemetry.addData("angle 3:", angles[2]);
            telemetry.update();
        }
    }

    public static void getHeading(double[] angles, BNO055IMU imu, AngleUnit angleUnit) {
        Orientation anglesOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, angleUnit);
        angles[0] = anglesOrientation.firstAngle;
        angles[1] = anglesOrientation.secondAngle;
        angles[2] = anglesOrientation.thirdAngle;
    }
    public void Drive(double vert, double horz, double rotate){
        double frdrive = -vert - horz - rotate;
        double fldrive = -vert + horz + rotate;
        double brdrive = -vert + horz - rotate;
        double bldrive = -vert - horz + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        frontRight.setPower(driveSpeed * Constants.driveTuningFR * frdrive / max);
        frontLeft.setPower(driveSpeed * Constants.driveTuningFL * fldrive / max);
        backRight.setPower(driveSpeed * Constants.driveTuningBR * brdrive / max);
        backLeft.setPower(driveSpeed * Constants.driveTuningBL * bldrive / max);
    }
}