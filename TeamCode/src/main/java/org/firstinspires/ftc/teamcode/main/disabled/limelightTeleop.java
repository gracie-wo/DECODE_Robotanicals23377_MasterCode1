package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled

@TeleOp(name = "LimeLight Teleop", group = "Tele Op")
public class limelightTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        LLResult llResult = limelight.getLatestResult();

        Servo rotator = hardwareMap.get(Servo.class, "rotator");
//        DcMotor rotate = hardwareMap.dcMotor.get("rotate");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        boolean camera_on = false;
        //basically center = 0.8
        //range rn: 0.25 - 1
        double launchPosition = 0.8;

        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();

            if(gamepad2.right_bumper){
                rotator.setPosition(launchPosition);
                camera_on = true;
            }

            if (camera_on && llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Distance", getDistanceFromTags(llResult.getTa()));
                telemetry.addData("Tx" , llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("BotPose", botPose.toString());
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                telemetry.update();
            }

            if(gamepad2.left_bumper){
                camera_on = false;
                telemetry.clear();
                telemetry.addData("Camera:", "Off");
                telemetry.update();
            }


            if(camera_on && llResult.getTx() < -1){
                if(launchPosition > 0.25){
                    launchPosition -= 0.0001;
                    rotator.setPosition(launchPosition);
                }
            }

            if(camera_on && llResult.getTx() > 1){
                if(launchPosition < 1){
                    launchPosition += 0.0001;
                    rotator.setPosition(launchPosition);
                }
            }

            if(gamepad2.right_bumper){
                rotator.setPosition(1);
            }

            if(gamepad2.left_bumper){
                rotator.setPosition(0);
            }

//            if (camera_on && llResult.getTx() < -5){
//                if(rotate.getCurrentPosition() < 0){
//                    rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    rotate.setTargetPosition(0);
//                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rotate.setPower(0.5);
//                } else {
//                    rotate.setPower(0.2);
//                }
//            }
//
//            if (camera_on && llResult.getTx() > 5){
//                if(rotate.getCurrentPosition() >= 537){
//                    rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    rotate.setTargetPosition(0);
//                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rotate.setPower(0.5);
//                } else {
//                    rotate.setPower(0.2);
//                }
//            }


        }


    }

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta);
        return distance;
    }
}