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

            telemetry.addData("Autonomous", "Selections");
            telemetry.addData("Alliance", isRed ? "Red" : "Blue");
            telemetry.addData("Near Crater", nearCrater ? "Yes" : "No");
            telemetry.addData("Deploy From Lander", deployFromLander ? "Yes" : "No");

            boolean selectionsAreGood = myAO.selectionsGood();
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