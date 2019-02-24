package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.PidUdpReceiver;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous

public class StevesPIDTuner extends NostromoAutonomousMethods{

   // public NostromoBotMotorDumper robot = new NostromoBotMotorDumper();


    private PidUdpReceiver pidUdpReceiver;
    private RobotTelemetryDisplay rtd;
    private double kP, kI, kD;


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Create new instance of robot telemetry display
        rtd = new RobotTelemetryDisplay(this,"Nostromo");

        // Create new instance of receiver
        pidUdpReceiver = new PidUdpReceiver();

        // Start listening
        pidUdpReceiver.beginListening();

        // initialize the robot
     //   robot.initialize(hardwareMap);

        // set the transmission interval to 50 milliseconds
        telemetry.setMsTransmissionInterval(50);

        // Wait for start to be pushed
        waitForStart();

        while (opModeIsActive()) {

            updatePIDCoefficients();

            rtd.displayRobotTelemetry("Steves PID Tuner");

            rtd.displayRobotTelemetry("kP", String.valueOf(kP));
            rtd.displayRobotTelemetry("ki", String.valueOf(kI));
            rtd.displayRobotTelemetry("kD", String.valueOf(kD));
        }

        pidUdpReceiver.shutdown();
    }

    public void updatePIDCoefficients() {

        kP = pidUdpReceiver.getP();
        kI = pidUdpReceiver.getI();
        kD = pidUdpReceiver.getD();

        rtd.displayRobotTelemetry("kP",String.valueOf(kP));
    }
}
