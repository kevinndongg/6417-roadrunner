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

    double vert,horz,rotate;
    double relativeAngle,leftStickAngle, driveAngle, arcTan;
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

        double angleOffset = imu.getAngularOrientation().thirdAngle;

        // servo declarations

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            // drive calculations
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            driveAngle = (leftStickAngle - relativeAngle + 90) % 360;

            relativeAngle = (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 90) % 360;
            arcTan = (Math.toDegrees(Math.atan(vert / horz))) % 360;

            if(horz < 0) {
                leftStickAngle = arcTan + 180;
            } else if(vert < 0) {
                leftStickAngle = arcTan + 360;
            } else {
                leftStickAngle = arcTan;
            }

            Drive(vert, horz, rotate, leftStickAngle, relativeAngle);

            telemetry.addData("relative angle", relativeAngle);
            telemetry.addData("left stick angle", leftStickAngle);
            telemetry.addData("drive angle", driveAngle);
            telemetry.update();
        }
    }

    public void Drive(double vert, double horz, double rotate, double leftStickAngle, double relativeAngle){
        double driveAngle = leftStickAngle - relativeAngle -90;
        /*double frdrive = vert - horz - rotate;
        double fldrive = vert + horz + rotate;
        double brdrive = vert + horz - rotate;
        double bldrive = vert - horz + rotate;*/

        double frdrive = -rotate;
        double fldrive = + rotate;
        double brdrive = - rotate;
        double bldrive = + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        frontRight.setPower(driveSpeed * Constants.driveTuningFR * frdrive / max);
        frontLeft.setPower(driveSpeed * Constants.driveTuningFL * fldrive / max);
        backRight.setPower(driveSpeed * Constants.driveTuningBR * brdrive / max);
        backLeft.setPower(driveSpeed * Constants.driveTuningBL * bldrive / max);
    }
}