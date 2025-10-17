package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Testing: Boots", group = "Testing")
public class TestingBoots extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
//        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//
//        CRServo sweeperR = hardwareMap.get(CRServo.class, "sweeper1");
//        CRServo sweeperL = hardwareMap.get(CRServo.class, "sweeper2");

        DcMotor boot = hardwareMap.dcMotor.get("boot");
        boot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                boot.setPower(0.89);
            }

            if(gamepad1.b){
                boot.setPower(0);
            }
//            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = -gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x;
//
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            if (gamepad2.options){
//                imu.resetYaw();
//            }
//
//            double botHeading = 0.0;
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeft.setPower(frontLeftPower* 1);
//            backLeft.setPower(backLeftPower* 1);
//            frontRight.setPower(frontRightPower* 1);
//            backRight.setPower(backRightPower* 1);
//
//            if(gamepad1.a) {
//                sweeperR.setPower(1);
////                sweeperL.setPower(-1);
//            }
//
//            if(gamepad1.b){
//                sweeperR.setPower(0.0);
////                sweeperL.setPower(0.0);
//            }
//
//            if(gamepad1.x){
//                sweeperR.setPower(-1);
////                sweeperL.setPower(1);
//            }

        }


    }
}