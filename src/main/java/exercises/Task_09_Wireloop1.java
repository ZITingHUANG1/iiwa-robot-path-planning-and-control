package exercises;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.FileLogger;
import utility.SingleInstanceChecker;

import java.util.ArrayList;
import java.util.List;


public class Task_09_Wireloop1 extends RoboticsAPIApplication {

	public Task_09_Wireloop1(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();


		// initialization
		Task_09_Wireloop1 app = new Task_09_Wireloop1(
				RoboticsAPIContext.createFromResource(Task_09_Wireloop1.class, "RoboticsAPI.config.xml"));
		SunriseConnector.initialize(app);

		SunriseConnector.createInfoGui();

		SunriseConnector.getControlGui().setNextFrameName("wireloop_start");

		app.run();

		ExecutionController.waitForAllMotionsFinished();

		SunriseConnector.stopInfoGui();
		System.exit(0);
	}

	/**
	 * This class can be used to track the movement direction during the exploration. It can be set to POSITIVE and
	 * NEGATIVE and be toggled between both. Using value() returns the 1 or -1, respectively.
	 *
	 * @author seid_da
	 */
	public static class Direction {

		public static final int POSITIVE = 1;
		public static final int NEGATIVE = -1;

		private int value;


		private Direction(int value) {
			this.value = value;
		}

		/**
		 * Create a new Direction instace with POSITIVE value.
		 *
		 * @return Direction instance with value() returning 1.
		 */
		public static Direction POSITIVE() {
			return new Direction(POSITIVE);
		}

		/**
		 * Create a new Direction instace with NEGATIVE value.
		 *
		 * @return Direction instance with value() returning -1.
		 */
		public static Direction NEGATIVE() {
			return new Direction(NEGATIVE);
		}

		/**
		 * Toggle the value of this Direction instance between 1 and -1.
		 */
		public void invert() {
			this.value *= -1;
		}

		/**
		 * Set the value of this Direction instance to 1.
		 */
		public void setPositive() {
			this.value = 1;
		}

		/**
		 * Set the value of this Direction instance to -1.
		 */
		public void setNegative() {
			this.value = -1;
		}

		/**
		 * Returns the current direction as an integer factor (1 or -1).
		 *
		 * @return Signed int representing the direction.
		 */
		public final int value() {
			return value;
		}

		/**
		 * Returns the String representation of the current direction.
		 */
		public String toString() {
			return "" + value;
		}
	}

	private Direction xDirection = Direction.POSITIVE();
	private Direction yDirection = Direction.POSITIVE();

	private final double lowCartVel = 100;
	private final double highCartVel = 200;

	private final double moveUpDownDistance = 30;

	private Frame wireloopStart = null;

	private CartesianImpedanceControlMode cicm;

	LBR robot;
	Tool tool;
	ObjectFrame toolFrame;

	private void moveUp() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}


	private void moveDown() {
		RelativeLIN motion = BasicMotions.linRel(0, 0, -moveUpDownDistance, robot.getRootFrame());
		tool.move(motion.setCartVelocity(highCartVel));
	}

	@Override
	public void run() {
///////////////////////////////////////
		List<Frame> logfram = new ArrayList<>();
		//////////////////////
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** load start position [perviously teached] and define starting direction */
		//wireloopStart = DataHandler.loadFrame("wireloop_start").setBetaRad(0).setGammaRad(Math.PI);
		wireloopStart = DataHandler.loadFrame("F14").setBetaRad(0).setGammaRad(Math.PI);
		// xDirection = Direction.NEGATIVE();
		xDirection = Direction.POSITIVE();

		/** configure impedance control mode */
		cicm = new CartesianImpedanceControlMode();
		cicm.parametrize(CartDOF.TRANSL).setStiffness(2500);
		cicm.parametrize(CartDOF.ROT).setStiffness(300);

		/** break condition which stops at 10% of a joints maximum torque */
		ICondition cond = SunriseConnector.createJointTorqueCondition(0.02);
		IMotionContainer mc = null;
		IMotionContainer mcc = null;
		double force;

		Frame firstFrame = new Frame(Transformation.ofDeg(400, 300, 350, 180, 0, -180));
		Frame secondFrame = new Frame(Transformation.ofDeg(400, -300, 350, 180, 0, -180));
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();
		double cartVelocity = 150;

		/** for safety, move up */
		try {
			moveUp();
		} catch (Exception e) {
			SunriseConnector.goFront();
		}

		/** move to start position in two steps: first to the position 5 cm above the start and then vertical */
		Frame aboveWireloopStart = new Frame(Transformation.ofTranslation(0, 0, 50)
				.compose(wireloopStart.getTransformationProvider().getTransformation()));
		tool.moveAsync((BasicMotions.ptp(aboveWireloopStart).setJointVelocityRel(0.6).setBlendingCart(10)));
		tool.move(BasicMotions.lin(wireloopStart).setCartVelocity(150));

		/** insert code to move along the course here */
		int a = 1;
		double ang = 3.14159 * 10 / 180;
		boolean wrong_way = false;

		//List<Frame> savedFrames = [ [X=543.46 Y=-11.40 Z=-126.21 A=1.59 B=-0.00 C=-3.14],  [X=543.41 Y=3.68 Z=-126.07 A=1.59 B=-0.00 C=-3.14],  [X=543.11 Y=18.55 Z=-125.98 A=1.59 B=-0.00 C=-3.14],  [X=542.80 Y=33.57 Z=-125.85 A=1.59 B=-0.00 C=-3.14],  [X=542.48 Y=48.50 Z=-125.76 A=1.59 B=-0.00 C=-3.14],  [X=545.56 Y=63.15 Z=-126.49 A=1.42 B=-0.00 C=-3.13],  [X=550.98 Y=77.23 Z=-124.45 A=1.24 B=-0.00 C=-3.13],  [X=559.53 Y=87.45 Z=-125.72 A=-2.07 B=-0.00 C=3.12],  [X=571.73 Y=97.23 Z=-126.82 A=-2.42 B=-0.01 C=3.12],  [X=585.23 Y=104.93 Z=-125.06 A=-2.60 B=-0.01 C=3.12],  [X=599.92 Y=109.83 Z=-126.43 A=-2.77 B=-0.02 C=3.13],  [X=606.11 Y=95.94 Z=-127.68 A=-2.74 B=-0.02 C=3.12],  [X=620.20 Y=101.60 Z=-124.77 A=-2.74 B=-0.02 C=3.12],  [X=635.92 Y=99.60 Z=-126.15 A=3.02 B=-0.03 C=3.13],  [X=650.90 Y=94.96 Z=-124.48 A=2.84 B=-0.03 C=3.14],  [X=663.54 Y=85.85 Z=-126.07 A=2.49 B=-0.04 C=-3.13],  [X=673.77 Y=74.71 Z=-124.49 A=2.32 B=-0.03 C=-3.12],  [X=680.02 Y=60.80 Z=-125.73 A=1.97 B=-0.03 C=-3.11],  [X=683.72 Y=46.24 Z=-126.90 A=1.79 B=-0.02 C=-3.11],  [X=685.16 Y=31.18 Z=-124.97 A=1.62 B=-0.02 C=-3.10]];


		boolean startloop =false;

		while (true) {
			if(startloop&&robot.getCurrentCartesianPosition(toolFrame).distanceTo(wireloopStart)<30){
				break;
			}


			if (!SunriseConnector.getCabinet().getExecutionService().isPaused()) {
				/** move robot to right side */
				RelativeLIN motion = BasicMotions.linRel(15 * a, 0, 0, toolFrame).setMode(cartImpMode);
				mcc = tool.move(motion.setCartVelocity(highCartVel).breakWhen(cond));

				if (robot.getCurrentCartesianPosition(toolFrame).getZ() - wireloopStart.getZ() <= 1) {
					System.out.println("IN THE TABLE!!!!!!!!!");
					tool.move(BasicMotions.linRel(0, 0, -3, toolFrame).setMode(cartImpMode));
				} else if (robot.getCurrentCartesianPosition(toolFrame).getZ() - wireloopStart.getZ() >= 6) {
					System.out.println("out THE TABLE!!!!!!!!!");
					tool.move(BasicMotions.linRel(0, 0, 3, toolFrame).setMode(cartImpMode));
				}

				double jointval = robot.getCurrentJointPosition().get(JointEnum.J7);
				System.out.println("joint val is : " + jointval);

				if (jointval >= 3.14159 * 150 / 180) {
					System.out.println("Out of the joint limit");
					moveUp();
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					tool.move(BasicMotions.linRel(0, 0, 0, -3.14159, 0, 0, toolFrame).setMode(cartImpMode));
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					moveDown();
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					a = -a;
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
				} else if (jointval <= -3.14159 * 150 / 180) {
					System.out.println("Out of the negative joint limit");
					moveUp();
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					tool.move(BasicMotions.linRel(0, 0, 0, 3.14159, 0, 0, toolFrame).setMode(cartImpMode));
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					moveDown();
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					a = -a;
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
				}

				/** check if movement was aborted by break condition */
				if (mcc.hasFired(cond)) {
					startloop=true;
					//move back and try another angle
					tool.move(BasicMotions.lin(logfram.get(logfram.size()-1)));
					force = robot.getExternalForceTorque(toolFrame).getTorque().getZ();

					if (force > 0) {
						RelativeLIN motion1 = BasicMotions.linRel(0, 0, 0, -ang, 0, 0, toolFrame);
						tool.move(motion1.setCartVelocity(highCartVel).breakWhen(cond));
						System.out.println("force is positive!");
						System.out.println(ang);
						System.out.println(force);
					} else if (force < 0) {
						RelativeLIN motion2 = BasicMotions.linRel(0, 0, 0, ang, 0, 0, toolFrame);
						tool.move(motion2.setCartVelocity(highCartVel).breakWhen(cond));
						System.out.println("force is negative!");
						System.out.println(ang);
						System.out.println(force);
					} else {
						System.out.println("Error!!!");
					}

					if(ang > 3.14159 * 30/ 180) {

						System.out.println(ang );
						System.out.println(wrong_way);
						wrong_way=true;
						ang = 0;
					}
					else if(ang < 3.14159 * -30/ 180) {
						RelativeLIN motion1 = BasicMotions.linRel(0, 15*a, 0, 0, 0, 0, toolFrame);
						tool.move(motion1.setCartVelocity(highCartVel).setMode(cartImpMode));
						logfram.add(robot.getCurrentCartesianPosition(toolFrame));
						System.out.println(ang );
						System.out.println(wrong_way);
						wrong_way=false;
						ang = 0;
					}

					if(wrong_way) {
						ang -= 3.14159 * 10 / 180;
					}
					else{
						ang += 3.14159 * 10 / 180;
					}
				}
				else {
					logfram.add(robot.getCurrentCartesianPosition(toolFrame));
					ang = 3.14159 * 10 / 180;
					wrong_way = false;
					System.out.println(logfram);

					ThreadUtil.milliSleep(100);
				}
			}
		}
		System.out.println("training finished!!!!!!!!!!!!!!");
		while (true){
			for (Frame f:logfram
				 ) {
				tool.move(BasicMotions.lin(f));

			}
		}

	}
}
