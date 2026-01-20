package org.firstinspires.ftc.teamcode.main.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

//blue april tag
@Config
@Autonomous(name = "TESTING_limelight", group = "testing")
public class TESTING_limelightAuto extends LinearOpMode {

    //limelight
    String pattern = "LLL";

    //MAY CAUSE ERRORS
    private Limelight3A limelight;
    private LLResult llResult;


    //------------------------------------LIMELIGHT-----------------------------------------
    public class Limelight{
        public Limelight(HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(2);
            llResult = limelight.getLatestResult();
            limelight.start();
        }

        public class MoasicDetect implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int n = 0;
                while(n <= 10) {
                    llResult = limelight.getLatestResult();

                    int tagId = 21;

                    if (llResult != null && llResult.isValid()) {
                        Pose3D botPose = llResult.getBotpose_MT2();
                        telemetry.addData("Distance", getDistanceFromTags(llResult.getTa()));
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        telemetry.addData("BotPose", botPose.toString());
//                      telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                        telemetry.update();
                        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            // This is the AprilTag ID
                            tagId = (int) fiducial.getFiducialId();

                            if (tagId == 21) {
                                telemetry.addData("Detected Tag ID", "GPP");
                            } else if (tagId == 22) {
                                telemetry.addData("Detected Tag ID", "PGP");
                            } else if (tagId == 23) {
                                telemetry.addData("Detected Tag ID", "PPG");
                            }


                            telemetry.update();
                        }
                    }

                    n++;
                }

                return false;
            }
        }

        public Action mosaicDetect(){
            return new MoasicDetect();
        }
     }

     //------------------------------------MOTORS--------------------------------------------

    public class Launcher {
        private DcMotorEx launcher;

        private double voltChange;
        private VoltageSensor controlHubVoltageSensor;
        private double launchPower;

//         final double NEWR_P = 5.0;
//         final double NEWR_I = 0.2;
//         final double NEWR_D = 0.7;
//         final double NEWR_F = 11;

        public Launcher(HardwareMap hardwareMap){
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//             PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
//            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);
            controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            voltChange = voltSpeed(controlHubVoltageSensor);
            launchPower = (0.0025 * 175) + 0.05;
        }

        public class LaunchOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                llResult = limelight.getLatestResult();
                voltChange = voltSpeed(controlHubVoltageSensor);
//
                if(llResult != null && llResult.isValid()){
                    double distance = getDistanceFromTags(llResult.getTa());
                    launchPower = (0.0025 * (distance)) + voltChange;
                } else {
                    launchPower = (0.0025 * 175) + voltChange;
                }
                //185
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOn() {
            return new LaunchOn();
        }

        public class LaunchOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setPower(0);
                return false;
            }
        }

        public Action launchOff() {
            return new LaunchOff();
        }
    }

    //----------------------------SERVOS---------------------------------------------------
//    public class Spindex {
//        private Servo spindex;
//
//        int linePickUp;
//
//        public Spindex(HardwareMap hardwareMap){
//            spindex = hardwareMap.get(Servo.class, "spindex");
//            linePickUp = 1;
//            //values for pickup (1, 2, 3): 1, 0.1, 0.56
//        }
//
//        //intake
//        public class SpindexIntakeOne implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                if(linePickUp == 1){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(1);
//                        return false;
//                    }
//                } else if(linePickUp == 2){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    }
//                } else if(linePickUp == 3){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    }
//                }
//
//                spindex.setPosition(0.1);
//                return false;
//            }
//        }
//
//        public Action spindexIntakeOne(){
//            return new SpindexIntakeOne();
//        }
//
//
//        public class SpindexIntakeTwo implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                if(linePickUp == 1){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    }
//                } else if(linePickUp == 2){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    }
//                } else if(linePickUp == 3){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    }
//                }
//
//                spindex.setPosition(0.56);
//                return false;
//            }
//        }
//
//        public Action spindexIntakeTwo(){
//            return new SpindexIntakeTwo();
//        }
//
//
//        public class SpindexIntakeThree implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                if(linePickUp == 1){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(0.56);
//                        return false;
//                    }
//
//                    linePickUp = 2;
//                } else if(linePickUp == 2){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(1);
//                        return false;
//                    }
//
//                    linePickUp = 3;
//                } else if(linePickUp == 3){
//                    if(pattern.equals("GPP")){
//                        spindex.setPosition(0.1);
//                        return false;
//                    } else if(pattern.equals("PGP")){
//                        spindex.setPosition(1);
//                        return false;
//                    } else if(pattern.equals("PPG")){
//                        spindex.setPosition(1);
//                        return false;
//                    }
//                }
//
//                spindex.setPosition(1);
//                return false;
//            }
//        }
//
//        public Action spindexIntakeThree(){
//            return new SpindexIntakeThree();
//        }
//
//        //launch
//        public class SpindexLaunchOne implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                spindex.setPosition(0);
//                return false;
//            }
//        }
//
//        public Action spindexLaunchOne(){
//            return new SpindexLaunchOne();
//        }
//
//        public class SpindexLaunchTwo implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                spindex.setPosition(0.43);
//                return false;
//            }
//        }
//
//        public Action spindexLaunchTwo(){
//            return new SpindexLaunchTwo();
//        }
//
//        public class SpindexLaunchThree implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                spindex.setPosition(0.87);
//                return false;
//            }
//        }
//
//        public Action spindexLaunchThree(){
//            return new SpindexLaunchThree();
//        }
//    }

//    public class Rotator {
//        private Servo rotator;
//        private double launchPosition;
//        private boolean adjusted;
//
//        public Rotator(HardwareMap hardwareMap){
//            rotator = hardwareMap.get(Servo.class, "rotator");
//            launchPosition = 0.5;
//            adjusted = false;
//        }
//
//        public class Rotate implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                //find usual position
//                launchPosition = 0.5;
//                rotator.setPosition(launchPosition);
//                adjusted = false;
//
//                while (!adjusted) {
//                    llResult = limelight.getLatestResult();
//
//                    if(llResult != null && llResult.isValid()){
//                        if(llResult.getTx() < -1){
//                            if(launchPosition > 0.25){
//--------------------------------CHANGE----------------------------------------------
//                                launchPosition -= 0.0001;
//                                rotator.setPosition(launchPosition);
//                            }
//                        } else if(llResult.getTx() > 1){
//                            if(launchPosition < 1){
//                                launchPosition += 0.0001;
//                                rotator.setPosition(launchPosition);
//                            }
//                        } else {
//                            adjusted = true;
//                        }
//                    }
//                }
//                return false;
//            }
//        }
//
//        public Action rotate(){
//            return new Rotate();
//        }
//    }

    @Override
    public void runOpMode() {

        //instantiate at (0,0)
//        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Limelight limelight = new Limelight(hardwareMap);

//        Rotator rotator = new Rotator(hardwareMap);


//        TrajectoryActionBuilder nothing = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading (new Vector2d(0, 25));


//        Action move = nothing.build();

        waitForStart();
        if (isStopRequested()) return;


        // ------------------------- RUN AUTO -------------------------
        Actions.runBlocking(
                new SequentialAction(
                        limelight.mosaicDetect(),
//                        rotator.rotate(),
                        new SleepAction(30)
                )
        );


    }

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta) ;
        return distance;
    }

    public double voltSpeed(VoltageSensor controlHubVoltageSensor){
        double voltage = controlHubVoltageSensor.getVoltage();

        if(voltage >= 13.1){
            return 0.05;
        } else if (voltage >= 12.6){
            return 0.1;
        } else if (voltage >= 12.1){
            return 0.125;
        } else if (voltage >= 11.6){
            return 0.15;
        } else if (voltage >= 11.1){
            return 0.175;
        } else if (voltage >= 10.6){
            return 0.2;
        } else {
            return 0.05;
        }
    }
}