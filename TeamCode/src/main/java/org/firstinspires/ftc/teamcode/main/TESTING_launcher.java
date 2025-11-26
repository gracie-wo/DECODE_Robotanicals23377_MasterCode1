package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "two launchers", group = "Testing")
public class TESTING_launcher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor launchRight = hardwareMap.dcMotor.get("launchRight");
        DcMotor launchLeft = hardwareMap.dcMotor.get("launchLeft");
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                launchRight.setPower(1);
                launchLeft.setPower(-1);
            }

            if(gamepad1.b){
                launchRight.setPower(0);
                launchLeft.setPower(0);
            }

            if(gamepad1.x){
                launchRight.setPower(-1);
                launchLeft.setPower(1);
            }
        }


    }
}