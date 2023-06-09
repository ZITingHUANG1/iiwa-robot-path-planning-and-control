package robot;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;

//public class VoidApplication extends RoboticsAPIApplication {
public class VoidApplication extends RoboticsAPIApplication {

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		VoidApplication app = new VoidApplication();
		SunriseConnector.initialize(app);
		SunriseConnector.createInfoGui();

		app.run();
	}

	LBR robot;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();

		System.out.println("Starting " + this.getClass().getSimpleName());

		// System.out.println("MeasuredTorque: " + robot.getSensorForMeasuredTorque().getSensorData());
		// System.out.println("ExternalTorque: " + robot.getSensorForExternalTorque().getSensorData());
		// System.out.println("ExternalForce: " + robot.getSensorForExternalForce().getSensorData());

		// RobotController.getRobot()
		// .move(BasicMotions.ptp(new Frame(Transformation.ofDeg(700, 0, 250, -180, 0, 180))));

		// System.out.println(rc.getTool().getDefaultMotionFrame());
		System.out.println("TCP " + SunriseConnector.getTcpFrame());

		while (true)
			;
		// System.exit(0);
	}

}
