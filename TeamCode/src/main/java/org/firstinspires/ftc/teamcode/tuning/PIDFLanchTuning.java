package org.firstinspires.ftc.teamcode.tuning; // adjust to your teamcode package

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Config makes the variables below editable in the FTC Dashboard
@Config
@TeleOp(name = "Launcher PID Tuner OpMode", group = "Tuning")
public class PIDFLanchTuning extends LinearOpMode {

    // Declare public static variables for the Dashboard to access
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0; // Feedforward
    public static double TARGET_VELOCITY = 1500; // Target velocity in ticks/sec

    private DcMotorEx launcherMotor;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;

    // Define a simple custom PID Controller class within the OpMode for simplicity
    public class SimplePIDController {
        public double update(double reference, double state) {
            double error = reference - state;
            integralSum += error * timer.seconds(); // Integrate error over time
            double derivative = (error - lastError) / timer.seconds(); // Calculate derivative of error
            lastError = error;
            timer.reset(); // Reset the timer for the next loop iteration

            // Calculate output power using PIDF formula
            double output = (error * kP) + (derivative * kD) +
                    (integralSum * kI) + (reference * kF);
            return output;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher"); // Name in hardware map
        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // We use RUN_WITHOUT_ENCODER because we are implementing our own PID control loop
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // Adjust as needed

        // Initialize FtcDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SimplePIDController pidController = new SimplePIDController();

        waitForStart();

        while (opModeIsActive()) {
            // Get current velocity (ticks per second)
            double currentVelocity = launcherMotor.getVelocity();

            // Calculate motor power using the PID controller
            double power = pidController.update(TARGET_VELOCITY, currentVelocity);
            launcherMotor.setVelocity(power);

            // Send data to the dashboard
            telemetry.addData("Target Velocity (tps)", TARGET_VELOCITY);
            telemetry.addData("Actual Velocity (tps)", currentVelocity);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}