package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main", group = "Main")
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        CRServo sweeper1R = hardwareMap.get(CRServo.class, "sweeper1");
        CRServo sweeper1L = hardwareMap.get(CRServo.class, "sweeper2");
        CRServo sweeper2R = hardwareMap.get(CRServo.class, "sweeper3");
        CRServo sweeper2L = hardwareMap.get(CRServo.class, "sweeper4");
        CRServo sweeper3L = hardwareMap.get(CRServo.class, "sweeper5");
        CRServo sweeper3R = hardwareMap.get(CRServo.class, "sweeper6");

        double rotPos = 0.0;


        DcMotor launchRight = hardwareMap.dcMotor.get("launchRight");
        DcMotor launchLeft = hardwareMap.dcMotor.get("launchLeft");
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo rotator = hardwareMap.get(Servo.class, "rotator");

        final double NEWR_P = 0.0;
        final double NEWR_I = 0.0;
        final double NEWR_D = 0.0;
        final double NEWR_F = 0.0;

        DcMotorControllerEx motorControllerExR = (DcMotorControllerEx)launchRight.getController();
        int motorIndexR = ((DcMotorEx)launchRight).getPortNumber();

        PIDFCoefficients pidfOrigR = motorControllerExR.getPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
        motorControllerExR.setPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);

        PIDFCoefficients pidfModifiedR = motorControllerExR.getPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER);



        final double NEWL_P = 0.0;
        final double NEWL_I = 0.0;
        final double NEWL_D = 0.0;
        final double NEWL_F = 0.0;

        DcMotorControllerEx motorControllerExL = (DcMotorControllerEx)launchLeft.getController();
        int motorIndexL = ((DcMotorEx)launchLeft).getPortNumber();

        PIDFCoefficients pidfOrigL = motorControllerExL.getPIDFCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNewL = new PIDFCoefficients(NEWL_P, NEWL_I, NEWL_D, NEWL_F);
        motorControllerExL.setPIDFCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER, pidfNewL);

        PIDFCoefficients pidfModifiedL = motorControllerExL.getPIDFCoefficients(motorIndexL, DcMotor.RunMode.RUN_USING_ENCODER);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(44, 60.2);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad2.options){
                imu.resetYaw();
            }

            double botHeading = 0.0;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower* 1);
            backLeft.setPower(backLeftPower* 1);
            frontRight.setPower(frontRightPower* 1);
            backRight.setPower(backRightPower* 1);

            //intake on
            if(gamepad1.a){
                intake.setPower(1);

                sweeper1R.setPower(-0.45);
                sweeper1L.setPower(0.45);

                sweeper2R.setPower(-0.1);
                sweeper2L.setPower(0.1);

                sweeper3R.setPower(0.5);
                sweeper3L.setPower(-0.5);
            }

            //intake off
            if(gamepad1.y){
                intake.setPower(0);

                sweeper1R.setPower(0);
                sweeper1L.setPower(0);

                sweeper2R.setPower(0);
                sweeper2L.setPower(0);

                sweeper3R.setPower(0);
                sweeper3L.setPower(0);
            }

            //gamepad 2
            if(gamepad2.x){
                intake.setPower(1);

                sweeper1R.setPower(-1);
                sweeper1L.setPower(1);
                sweeper2R.setPower(-1);
                sweeper2L.setPower(1);
                sweeper3R.setPower(-1);
                sweeper3L.setPower(1);

                //launch from far away
//                launchRight.setPower(0.63);
//                launchLeft.setPower(0.63);

                //launch from close
                launchRight.setPower(0.45);
                launchLeft.setPower(0.45);
            }

            if(gamepad2.b){
                intake.setPower(0);

                sweeper1R.setPower(0);
                sweeper1L.setPower(0);
                sweeper2R.setPower(0);
                sweeper2L.setPower(0);
                sweeper3R.setPower(0);
                sweeper3L.setPower(0);

                launchRight.setPower(0);
                launchLeft.setPower(0);
            }

            if(gamepad2.dpad_up){
                rotator.setPosition(0);
            }


            if(gamepad2.dpad_down){
                rotator.setPosition(0.99);
            }

            if(gamepad2.dpad_left){
                if(rotPos > 0.0){
                    rotPos -= 0.001;
                    rotator.setPosition(rotPos);
                }

                telemetry.addData("Servo Position", rotator.getPosition());
                telemetry.update();
            }


            if (gamepad2.dpad_right) {
                if(rotPos < 0.99){
                    rotPos += 0.001;
                    rotator.setPosition(rotPos);
                }

                telemetry.addData("Servo Position", rotator.getPosition());
                telemetry.update();

            }

        }

    }
}