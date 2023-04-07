package exercises;
import java.util.Vector;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.FileLogger;
import utility.SingleInstanceChecker;
import utility.FileLogger.Fields;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

public class Task_04_JointAndTaskSpace extends RoboticsAPIApplication {
	
	public Task_04_JointAndTaskSpace(RoboticsAPIContext context){
		super(context); 
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_04_JointAndTaskSpace app = new Task_04_JointAndTaskSpace(RoboticsAPIContext.createFromResource(Task_04_JointAndTaskSpace.class, "RoboticsAPI.config.xml")); 
		SunriseConnector.initialize(app);

		app.run();
		
		ExecutionController.waitForAllMotionsFinished();
	}

	LBR robot;
	Tool tool;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		System.out.println("Starting " + this.getClass().getSimpleName());

		double blendingRadiusCart = 10;
		double jointVelocity = 0.2;

		Frame F0 = DataHandler.loadFrame("F5");
		Frame F1 = DataHandler.loadFrame("F6");
		Frame F2 = DataHandler.loadFrame("F0");
		Frame F3 = DataHandler.loadFrame("F2");
		JointPosition J0 = DataHandler.loadJointPos("J5");
		JointPosition J1 = DataHandler.loadJointPos("J6");
		JointPosition J2 = DataHandler.loadJointPos("J0");
		JointPosition J3 = DataHandler.loadJointPos("J2");
		
		/** ----- create a motion batch with PTP movements to joint positions ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2  */
		 Vector<RobotMotion<?>> motionVector = new Vector<RobotMotion<?>>();
		 motionVector.add(BasicMotions.ptp(J0));
		motionVector.add(BasicMotions.ptp(J1));
		motionVector.add(BasicMotions.ptp(J2));
		motionVector.add(BasicMotions.ptp(J3));
		 RobotMotion<?>[] motionArray = motionVector.toArray(new RobotMotion<?>[motionVector.size()]);

		 MotionBatch mb = new MotionBatch( motionArray);
		mb.setJointVelocityRel(jointVelocity);
		mb.setBlendingCart(blendingRadiusCart );
		

		/** use a sync move to the first (joint-)position, then start logging and execute the motion batch */

		FileLogger.startLogging("log_PTP_jp", Fields.TIME, Fields.TRANSLATIONAL_DISTANCE, Fields.ROTATIONAL_DISTANCE,
				Fields.JOINT_SPACE_DISTANCE);

		tool.move(BasicMotions.ptp(J0));


		double blendingRadiusJoints = 0.5;
		System.out.println("Moving to J0");
		tool.moveAsync(BasicMotions.ptp(J0));
		tool.moveAsync(BasicMotions.ptp(J0));
		System.out.println("Moving to J1 with blending");
		tool.moveAsync(BasicMotions.ptp(J1).setBlendingRel(blendingRadiusJoints));
		System.out.println("Moving to J2");
		tool.moveAsync(BasicMotions.ptp(J2).setBlendingRel(blendingRadiusJoints));
		System.out.println("Moving to J3");
		tool.moveAsync(BasicMotions.ptp(J3));
		System.out.println("Moving to J0");
		tool.moveAsync(BasicMotions.ptp(J0));


		FileLogger.stopLogging();

		/** ----- create a motion batch with PTP movements to frames ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2 */

        /**
		Vector<RobotMotion<?>> motionVector = new Vector<RobotMotion<?>>();
		motionVector.add(BasicMotions.lin(F0));
		motionVector.add(BasicMotions.lin(F1));
		motionVector.add(BasicMotions.lin(F2));
		motionVector.add(BasicMotions.lin(F3));
		RobotMotion<?>[] motionArray = motionVector.toArray(new RobotMotion<?>[motionVector.size()]);*/




		/** ----- create a motion batch with LIN movements to frames ----- */
		/** add Cartesian blending with 10 mm and set joint velocity to 0.2 */

		/**
		 *
		 * tool.move(BasicMotions.ptp(J0));
		 * 		FileLogger.startLogging("log_PTP_jp", Fields.TIME, Fields.TRANSLATIONAL_DISTANCE, Fields.ROTATIONAL_DISTANCE,
		 * 				Fields.JOINT_SPACE_DISTANCE);
		 * 		tool.move(MotionBatch);
		 * 		FileLogger.stopLogging();
		System.out.println("Moving LIN to F0 with blending");
		tool.moveAsync(BasicMotions.lin(F0).setBlendingCart(blendingRadiusCart));
		System.out.println("Moving LIN to F0 with blending");
		tool.moveAsync(BasicMotions.lin(F1).setBlendingCart(blendingRadiusCart));
		System.out.println("Moving LIN to F0 with blending");
		tool.moveAsync(BasicMotions.lin(F2).setBlendingCart(blendingRadiusCart));
		System.out.println("Moving LIN to F0 with blending");
		tool.moveAsync(BasicMotions.lin(F3).setBlendingCart(blendingRadiusCart));*/
		
		

		/** wait until movements are finished [if there are async movements] */
		ExecutionController.waitForAllMotionsFinished();
		System.exit(0);
	}
}
