package exercises;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.DataHandler;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;


public class Task_08_Wireloop extends RoboticsAPIApplication {

	public Task_08_Wireloop(RoboticsAPIContext context) {
		super(context);
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();


		// initialization
		Task_08_Wireloop app = new Task_08_Wireloop(
				RoboticsAPIContext.createFromResource(Task_08_Wireloop.class, "RoboticsAPI.config.xml"));
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
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		toolFrame = tool.getDefaultMotionFrame();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** load start position [perviously teached] and define starting direction */
		//wireloopStart = DataHandler.loadFrame("wireloop_start").setBetaRad(0).setGammaRad(Math.PI);
		wireloopStart = DataHandler.loadFrame("F11").setBetaRad(0).setGammaRad(Math.PI);
		// xDirection = Direction.NEGATIVE();
		xDirection = Direction.POSITIVE();

		/** configure impedance control mode */
		cicm = new CartesianImpedanceControlMode();
		cicm.parametrize(CartDOF.TRANSL).setStiffness(2500);
		cicm.parametrize(CartDOF.ROT).setStiffness(300);

		/** break condition which stops at 10% of a joints maximum torque */
		////////////////////////////////////////////////////////////////////////////////////////////////!!!
		ICondition cond = SunriseConnector.createJointTorqueCondition(0.02);
		IMotionContainer mc = null;
		IMotionContainer mcc = null;
		double force;

		/////////////////////////////////////////
		Frame firstFrame = new Frame(Transformation.ofDeg(400, 300, 350, 180, 0, -180));
		Frame secondFrame = new Frame(Transformation.ofDeg(400, -300, 350, 180, 0, -180));
		CartesianImpedanceControlMode cartImpMode = new CartesianImpedanceControlMode();
		double cartVelocity = 150;
		//////////////////////////////////

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
		/**
		 Frame F0 = DataHandler.loadFrame("F9");
		 JointPosition J0 = DataHandler.loadJointPos("J9");
		 //System.out.println("Moving to F0");

		 tool.move(BasicMotions.ptp(F0)); */

		/** insert code to move along the course here */
		int a= 1;
		double ang=3.14159 *15/ 180;
		while (true) {


			if (!SunriseConnector.getCabinet().getExecutionService().isPaused()) {
				/** move robot to right side */
				RelativeLIN motion = BasicMotions.linRel(30*a, 0, 0, toolFrame).setMode(cartImpMode);
				mcc = tool.move(motion.setCartVelocity(highCartVel).breakWhen(cond));
				//mc = tool.move(BasicMotions.lin(wireloopStart).breakWhen(cond).setMode(cartImpMode).setCartVelocity(cartVelocity));

				if(robot.getCurrentCartesianPosition(toolFrame).getZ() - wireloopStart.getZ() <=1){
					System.out.println("IN THE TABLE!!!!!!!!!");
					tool.move(BasicMotions.linRel(0, 0, -3, toolFrame).setMode(cartImpMode));
				}

				else if(robot.getCurrentCartesianPosition(toolFrame).getZ() - wireloopStart.getZ() >=6){
					System.out.println("out THE TABLE!!!!!!!!!");
					tool.move(BasicMotions.linRel(0, 0, 3, toolFrame).setMode(cartImpMode));
				}

				double jointval = robot.getCurrentJointPosition().get(JointEnum.J7);
				System.out.println("joint val is : " + jointval);
				if (jointval >= 3.14159 * 150 / 180) {
					System.out.println("hhhhhhhhhhhhhhhhhhhhhhhh");
					moveUp();
					tool.move(BasicMotions.linRel(0, 0, 0,-3.14159,0,0, toolFrame).setMode(cartImpMode));
					moveDown();
					a=-a;
				}

				else if(jointval <= -3.14159 * 50 / 180) {
					System.out.println("??????????");
					moveUp();
					tool.move(BasicMotions.linRel(0, 0, 0,3.14159,0,0, toolFrame).setMode(cartImpMode));
					moveDown();
					a=-a;

				}

				/** check if movement was aborted by break condition */
				if (mcc.hasFired(cond)) {
					//tool.move(BasicMotions.lin(secondFrame).breakWhen(cond).setMode(cartImpMode).setCartVelocity(cartVelocity));

					force = robot.getExternalForceTorque(toolFrame).getTorque().getZ();
					//System.out.println("movement to right side aborted due to break condition. Force values: " + force);

						if (force > 0) {
							RelativeLIN motion1 = BasicMotions.linRel(0, 0, 0, ang, 0, 0, toolFrame);
							tool.move(motion1.setCartVelocity(highCartVel).breakWhen(cond));
						} else if (force < 0) {
							RelativeLIN motion2 = BasicMotions.linRel(0, 0, 0, -ang, 0, 0, toolFrame);
							tool.move(motion2.setCartVelocity(highCartVel).breakWhen(cond));
						}
						//tool.move(motion.setCartVelocity(highCartVel));


						else {
							System.out.println("Error!!!");
						}
					}

					ThreadUtil.milliSleep(100); // small sleep so that we do not spam new motions while in contact


					//break;
				}

			}
		}

}
