package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutoOptions.AutoOptions;

@Autonomous
@Disabled

public class StevesAutonomous extends RoboraiderAutonomous {

    private boolean isRed             = false;
    private boolean nearCrater        = false;
    private boolean deployFromLander  = false;
    private boolean selectionsAreGood = false;
    public ProtoBot robot = new ProtoBot();


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {


        // Ask drivers how they want autonomous to work
        AutoOptions myAO = new AutoOptions(this);

        // While the drivers haven't made up their mind, keep asking what they want to do
        while (!selectionsAreGood) {

            isRed            = myAO.selectAlliance();              // Get the alliance (Red or Blue)
            nearCrater       = myAO.selectLocation();              // Get where the robot is starting from (Depot or Crater)
            deployFromLander = myAO.selectDeployFromLander();      // Should the robot deploy from the lander (Yes or No)

            // Add new/additional auto options, so things like drive to depot, drop team marker, etc..
           // moveToDepot      = myAO.selectMoveToDepot();           // Should the robot go to depot
           //

            // Display the options selected
            // We show two options per line, to save space and lines.  The maximum number of characters
            // per line is roughly 45.  Maximum number of lines to be displayed is 9.
            telemetry.addLine().addData("Autonomous", "Selections");
            telemetry.addLine().addData("Alliance:", isRed ? "Red" : "Blue").addData("  Near Crater:", nearCrater ? "Yes" : "No");
            telemetry.addLine().addData("Deploy From Lander:", deployFromLander ? "Yes" : "No");

            // Verify that the autonomous selections are good, if so we are ready to rumble.  If not, well ask again.
            // Note: Normally when data is shown on the drive station, a telemetry.update() is done.  In this case
            //       we do not issue the telemetry.update() since selectionsGood() will issue the telemetry.update()
            //       displaying the autonomous selections and then asking if the autonomous selections are good.
            selectionsAreGood = myAO.selectionsGood();
        }

        robot.initialize(hardwareMap);

        gamepad1.reset();


        telemetry.addLine("Initialized");
        telemetry.update();

        // Wait for start to be pushed
        waitForStart();

        // Deploy From Lander
        if (deployFromLander) {
            DeployRobot(robot);
        }

        // Move to Depot
        if (moveToDepot) {
            if (nearCrater) {
                moveDepotFromCraterStart(robot);
            }
            if (!nearCrater) {
                moveDepotFromDepotStart(robot);
            }

        }
    }


}