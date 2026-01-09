package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="limelight", group="Testing")
public class TESTING_AprilTagLimelight extends OpMode {
   private Limelight3A limelight;
   private IMU imu;

   private double distance;
   private int tickPosition;

    private DcMotor rotate;

    @Override
    public void init(){
//        rotate = hardwareMap.dcMotor.get("rotate");
//
//        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rotate.setTargetPosition(268);
//        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        tickPosition = 268;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); //april tag #20 pipeline
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTags(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx" , llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
//            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        }
//        if (llResult.getTx() < -10){ //FIND THE RANGE!!!!
//            tickPosition -= 1;
//            rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rotate.setTargetPosition(tickPosition);
//            rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotate.setPower(0.5);
//        }
//
//        if (llResult.getTx() > 10){
//            tickPosition += 1;
//            rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rotate.setTargetPosition(tickPosition);
//            rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotate.setPower(0.5);
//        }
    }

    public double getDistanceFromTags(double ta){
        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta);
        return distance;
    }
}
