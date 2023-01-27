package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Methods {
    public static double grabberClose() {
        return 0.34;
    }

    public static double grabberOpen() {
        return 0.52;
    }

    public static double wristDown() {
        return 0.76;
    }

    public static double wristUp() {
        return 0.13;
    }

    public static int slideLow() {
        return 300;
    }

    public static int slideMedium() {return 1150;}

    public static double driveSpeedNormal() {return 0.6;}

    public static double driveSpeedSlow() {return 0.3;}

    public static double driveSpeedFast() {return 0.9;}
}
