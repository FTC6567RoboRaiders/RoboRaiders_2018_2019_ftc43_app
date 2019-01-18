package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.AutoOptions.AutoOptions;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous
@Disabled
public class StevesStrafing extends NostromoAutonomousMethods{

    private boolean isRed             = false;
    private boolean startLocation     = false;
    private boolean deployFromLander  = false;
    private boolean claimDepot        = false;
    private boolean parkInCrater      = false;
    private boolean selectionsAreGood = false;
    private boolean sampling     = false;
    public NostromoBot robot = new NostromoBot();




    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        // Wait for start to be pushed
        waitForStart();

        while (opModeIsActive()) {
            encodersMove(robot,24,1,"right")
        }
    }
}
