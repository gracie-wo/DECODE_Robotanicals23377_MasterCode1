package org.firstinspires.ftc.teamcode.main.OldAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Disabled
@Autonomous(name = "AutoRedBackShootOld", group = "Autonomous")
public class RedBackShootOld extends LinearOpMode {

    //------------------------------------MOTORS--------------------------------------------
    public class Intake {
        private DcMotorEx intake;

        public Intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class IntakeOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }

        public Action intakeOn() {
            return new IntakeOn();
        }

        public class IntakeOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }
    }

    public class LaunchRight {
        private DcMotorEx launchRight;

        public LaunchRight(HardwareMap hardwareMap){
            launchRight = hardwareMap.get(DcMotorEx.class, "launchRight");
            launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LaunchRightOnFar implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchRight.setPower(0.63);
                return false;
            }
        }

        public Action launchRightOnFar() {
            return new LaunchRightOnFar();
        }

        public class LaunchRightOnClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchRight.setPower(0.4);
                return false;
            }
        }

        public Action launchRightOnClose() {
            return new LaunchRightOnClose();
        }

        public class LaunchRightOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchRight.setPower(0);
                return false;
            }
        }

        public Action launchRightOff() {
            return new LaunchRightOff();
        }
    }

    public class LaunchLeft {
        private DcMotorEx launchLeft;

        public LaunchLeft(HardwareMap hardwareMap){
            launchLeft = hardwareMap.get(DcMotorEx.class, "launchLeft");
            launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LaunchLeftOnFar implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchLeft.setPower(0.63);
                return false;
            }
        }

        public Action launchLeftOnFar() {
            return new LaunchLeftOnFar();
        }

        public class LaunchLeftOnClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchLeft.setPower(0.4);
                return false;
            }
        }

        public Action launchLeftOnClose() {
            return new LaunchLeftOnClose();
        }

        public class LaunchLeftOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchLeft.setPower(0);
                return false;
            }
        }

        public Action launchLeftOff() {
            return new LaunchLeftOff();
        }
    }

    //----------------------------SERVOS---------------------------------------------------
    public class Sweeper1R {
        private CRServo sweeper1R;

        public Sweeper1R(HardwareMap hardwareMap){
            sweeper1R = hardwareMap.get(CRServo.class, "sweeper1");
        }

        public class Sweeper1RIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1R.setPower(-0.45);
                return false;
            }
        }

        public Action sweeper1RIntakeOn(){
            return new Sweeper1RIntakeOn();
        }


        public class Sweeper1RLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1R.setPower(-1);
                return false;
            }
        }

        public Action sweeper1RLaunchOn(){
            return new Sweeper1RLaunchOn();
        }


        public class Sweeper1ROff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1R.setPower(0);
                return false;
            }
        }

        public Action sweeper1ROff(){
            return new Sweeper1ROff();
        }
    }

    public class Sweeper1L {
        private CRServo sweeper1L;

        public Sweeper1L(HardwareMap hardwareMap){
            sweeper1L = hardwareMap.get(CRServo.class, "sweeper2");
        }

        public class Sweeper1LIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1L.setPower(0.45);
                return false;
            }
        }

        public Action sweeper1LIntakeOn(){
            return new Sweeper1LIntakeOn();
        }


        public class Sweeper1LLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1L.setPower(1);
                return false;
            }
        }

        public Action sweeper1LLaunchOn(){
            return new Sweeper1LLaunchOn();
        }


        public class Sweeper1LOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper1L.setPower(0);
                return false;
            }
        }

        public Action sweeper1LOff(){
            return new Sweeper1LOff();
        }
    }

    public class Sweeper2R {
        private CRServo sweeper2R;

        public Sweeper2R(HardwareMap hardwareMap){
            sweeper2R = hardwareMap.get(CRServo.class, "sweeper3");
        }

        public class Sweeper2RIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2R.setPower(-0.1);
                return false;
            }
        }

        public Action sweeper2RIntakeOn(){
            return new Sweeper2RIntakeOn();
        }


        public class Sweeper2RLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2R.setPower(-1);
                return false;
            }
        }

        public Action sweeper2RLaunchOn(){
            return new Sweeper2RLaunchOn();
        }


        public class Sweeper2ROff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2R.setPower(0);
                return false;
            }
        }

        public Action sweeper2ROff(){
            return new Sweeper2ROff();
        }
    }

    public class Sweeper2L {
        private CRServo sweeper2L;

        public Sweeper2L(HardwareMap hardwareMap){
            sweeper2L = hardwareMap.get(CRServo.class, "sweeper4");
        }

        public class Sweeper2LIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2L.setPower(0.1);
                return false;
            }
        }

        public Action sweeper2LIntakeOn(){
            return new Sweeper2LIntakeOn();
        }


        public class Sweeper2LLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2L.setPower(1);
                return false;
            }
        }

        public Action sweeper2LLaunchOn(){
            return new Sweeper2LLaunchOn();
        }


        public class Sweeper2LOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper2L.setPower(0);
                return false;
            }
        }

        public Action sweeper2LOff(){
            return new Sweeper2LOff();
        }
    }

    public class Sweeper3R {
        private CRServo sweeper3R;

        public Sweeper3R(HardwareMap hardwareMap){
            sweeper3R = hardwareMap.get(CRServo.class, "sweeper6");
        }

        public class Sweeper3RIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3R.setPower(0.5);
                return false;
            }
        }

        public Action sweeper3RIntakeOn(){
            return new Sweeper3RIntakeOn();
        }


        public class Sweeper3RLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3R.setPower(-1);
                return false;
            }
        }

        public Action sweeper3RLaunchOn(){
            return new Sweeper3RLaunchOn();
        }


        public class Sweeper3ROff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3R.setPower(0);
                return false;
            }
        }

        public Action sweeper3ROff(){
            return new Sweeper3ROff();
        }
    }

    public class Sweeper3L {
        private CRServo sweeper3L;

        public Sweeper3L(HardwareMap hardwareMap){
            sweeper3L = hardwareMap.get(CRServo.class, "sweeper5");
        }

        public class Sweeper3LIntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3L.setPower(-0.5);
                return false;
            }
        }

        public Action sweeper3LIntakeOn(){
            return new Sweeper3LIntakeOn();
        }


        public class Sweeper3LLaunchOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3L.setPower(1);
                return false;
            }
        }

        public Action sweeper3LLaunchOn(){
            return new Sweeper3LLaunchOn();
        }


        public class Sweeper3LOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sweeper3L.setPower(0);
                return false;
            }
        }

        public Action sweeper3LOff(){
            return new Sweeper3LOff();
        }
    }

    public class Rotator {
        private Servo rotator;

        public Rotator(HardwareMap hardwareMap){
            rotator = hardwareMap.get(Servo.class, "rotator");
        }

        public class RotatorFar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0.25);
                return false;
            }
        }

        public Action rotatorFar(){
            return new RotatorFar();
        }

        public class RotatorClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0.5);
                return false;
            }
        }

        public Action rotatorClose(){
            return new RotatorClose();
        }
    }

    @Override
    public void runOpMode() {

        //instantiate at (0,0)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Intake intake = new Intake(hardwareMap);
        LaunchRight launchRight = new LaunchRight(hardwareMap);
        LaunchLeft launchLeft = new LaunchLeft(hardwareMap);

        Sweeper1R sweeper1R = new Sweeper1R(hardwareMap);
        Sweeper1L sweeper1L = new Sweeper1L(hardwareMap);
        Sweeper2R sweeper2R = new Sweeper2R(hardwareMap);
        Sweeper2L sweeper2L = new Sweeper2L(hardwareMap);
        Sweeper3R sweeper3R = new Sweeper3R(hardwareMap);
        Sweeper3L sweeper3L = new Sweeper3L(hardwareMap);

        Rotator rotator = new Rotator(hardwareMap);

//-------------------------Build Pathways------------------------------------------
        TrajectoryActionBuilder toStart = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, 17));
        TrajectoryActionBuilder turn1 = toStart.fresh()
                .turn(Math.toRadians(-90));
        TrajectoryActionBuilder toBall1 = turn1.fresh()
                .strafeToConstantHeading(new Vector2d(25,17));
        TrajectoryActionBuilder ballRow1 = toBall1.fresh()
                .strafeToConstantHeading(new Vector2d(41,17));
        TrajectoryActionBuilder moveBack1 = ballRow1.fresh()
                .strafeToConstantHeading(new Vector2d(23,17));
        TrajectoryActionBuilder turn2 = moveBack1.fresh()
                .turn(Math.toRadians(90));
        TrajectoryActionBuilder toLaunch = turn2.fresh()
                .strafeToConstantHeading(new Vector2d(23,74));
        TrajectoryActionBuilder turnToLaunch = toLaunch.fresh()
                .turn(45);

//        TrajectoryActionBuilder toNextStep = toStart.fresh()
//                .strafeToLinearHeading(new Vector2d(0, 0), 0);

//-----------------------During INIT actions-------------------------------------
//        Actions.runBlocking(launchRight.launchRightOnFar());
//        Actions.runBlocking(launchLeft.launchLeftOnFar());

        waitForStart();

//-------------------------Build Actions---------------------------------------------
        Action to_start_action = toStart.build();
        Action turn1_action = turn1.build();
        Action to_ball_action = toBall1.build();
        Action get_ball_action = ballRow1.build();
        Action move_back_action = moveBack1.build();
        Action turn2_action = turn2.build();
        Action toLauncher_action = toLaunch.build();
        Action turn_to_launch = turnToLaunch.build();

//        Action to_next_step_action = toNextStep.build();


        if (isStopRequested()) return;

//-------------------------AUTO PATHWAYS---------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        to_start_action,
                        turn1_action,
                        to_ball_action,
                        new ParallelAction(
                                get_ball_action,
                                intake.intakeOn(),
                                sweeper1R.sweeper1RIntakeOn(),
                                sweeper1L.sweeper1LIntakeOn(),
                                sweeper2L.sweeper2LIntakeOn(),
                                sweeper2R.sweeper2RIntakeOn(),
                                sweeper3L.sweeper3LIntakeOn(),
                                sweeper3R.sweeper3RIntakeOn()
                        ),
                        new ParallelAction(
                                intake.intakeOff(),
                                sweeper1R.sweeper1ROff(),
                                sweeper1L.sweeper1LOff(),
                                sweeper2L.sweeper2LOff(),
                                sweeper2R.sweeper2ROff(),
                                sweeper3L.sweeper3LOff(),
                                sweeper3R.sweeper3ROff()
                        ),
                        move_back_action,
                        turn2_action,
                        toLauncher_action,
                        turn_to_launch
                )
        );

    }

}