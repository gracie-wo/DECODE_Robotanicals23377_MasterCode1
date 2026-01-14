package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Mosaic Limelight", group = "testing")
public class TESTING_limelight_mosaic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        LLResult llResult = limelight.getLatestResult();
        limelight.start();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            llResult = limelight.getLatestResult();

            int tagId = 21;

            if(llResult != null && llResult.isValid()){
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    // This is the AprilTag ID
                    tagId = (int) fiducial.getFiducialId();

                    if(tagId == 21){
                        telemetry.addData("Detected Tag ID", "GPP");
                    } else if(tagId == 22){
                        telemetry.addData("Detected Tag ID", "PGP");
                    } else if(tagId == 23){
                        telemetry.addData("Detected Tag ID", "PPG");
                    }

                    telemetry.update();
                }
            }




//            if (llResult != null && llResult.isValid()){
//                Pose3D botPose = llResult.getBotpose_MT2();
//                telemetry.addData("Tx" , llResult.getTx());
//                telemetry.addData("Ty", llResult.getTy());
//                telemetry.addData("Ta", llResult.getTa());
//                telemetry.addData("BotPose", botPose.toString());
////            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
//                telemetry.update();
//            }
        }

    }
}