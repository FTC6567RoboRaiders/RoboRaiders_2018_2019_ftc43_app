package RoboRaiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotTelemetryDisplay {

    /**
     * RobotTelemetryDisplay displays telemetry data of the robot.  This class assumes 9 lines per
     * display and 45 characters per line.
     *
     * Format for telemetry display:
     *
     * Line 1:    Title Line
     * Line 2:    Robot Status
     * Line 3-8:  Variable Section
     *
     * Example:
     * Line 1:    Nostromo Status
     * Line 2:    Robbot Status: Sampling
     * Line 3:    Drive Motor Power: 0.45
     * Line 4:    Heading: 95
     * Line 5:    Intake Arm: Extended
     * Line 6:    Encoder Value: 6784
     * Line 7:    Lift/Hang: Extended
     * Line 8:    Lift Claw: Closed
     *
     */

    OpMode op;
    String robotName;

    /**
     * Constructor for RobotTelemetryDisplay
     * @param op the OpMode tied to this class
     * @param robotName name of the robot
     */
    RobotTelemetryDisplay (OpMode op, String robotName) {
        this.op = op;
        this.robotName = robotName;
    }

    /**
     * will display the robot telemetry data as shown above
     * @param robotStatus - current robot status (e.g. Deployed, Sampling, etc.)
     * @param varInfo - string array of variable information that would be helpful to display, only the
     *                  first 6 elements will be displayed
     */
    public void displayRobotTelemetry(String robotStatus, StringBuilder[][] varInfo) {

        // Output the title line
        op.telemetry.addLine().addData(robotName, " Status");

        // Output the status line
        op.telemetry.addLine().addData("Status: ",robotStatus);

        // Output the variable information
        for (int i=0; i<6; i++) {
            //op.telemetry.addLine().addData(varInfo[i][0],varInfo[i][1].toString());
        }

        op.telemetry.update();

    }







}
