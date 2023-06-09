package utility;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;
import java.util.Map;

import robot.SunriseConnector;

import com.kuka.common.params.IParameter;
import com.kuka.common.params.IParameterSet;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;

public class DataHandler {

	/**
	 * Main path for storing and loading data.
	 */
	// public static final String DataPath = System.getenv("HOME") + "/RobotExercises";
	public static final String DataPath = "data";

	/**
	 * Not instantiable
	 */
	private DataHandler() {
	}

	public static String getNextFrameName() {
		String dirPath = DataHandler.DataPath + "/frames/";

		File fileObject = new File(dirPath);
		fileObject.mkdirs();

		ArrayList<String> names = new ArrayList<String>(Arrays.asList(fileObject.list()));

		int number = 0;
		for (int i = names.size() - 1; i >= 0; i--) {
			try {
				int n = Integer.parseInt(names.get(i).replace("F", "").replace(".frm", "")) + 1;
				if (n > number)
					number = n;
			} catch (Exception e) {
				continue;
			}
		}
		return ("F" + number);
	}

	/**
	 * Saves a frame in the frames folder of the configured data path.
	 * 
	 * @param name
	 *            Name of the file [without ending]
	 * @param frame
	 *            Frame to be saved
	 */
	public static void saveFrame(String name, Frame frame) {
		String filePath = DataHandler.DataPath + "/frames";

		File fileObject = new File(filePath);
		fileObject.mkdirs();

		System.out.println("Writing to file '" + filePath + "/" + name + ".frm" + "'");

		BufferedWriter writer = null;
		try {
			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filePath + "/" + name + ".frm"),
					"utf-8"));
			writer.write(String.format(Locale.US, "%.3f", frame.getX()));
			writer.write(String.format(Locale.US, " %.3f", frame.getY()));
			writer.write(String.format(Locale.US, " %.3f", frame.getZ()));
			writer.write(String.format(Locale.US, " %.3f", Math.toDegrees(frame.getAlphaRad())));
			writer.write(String.format(Locale.US, " %.3f", Math.toDegrees(frame.getBetaRad())));
			writer.write(String.format(Locale.US, " %.3f", Math.toDegrees(frame.getGammaRad())));

			writer.write("\n");

			Map<String, IRedundancyCollection> params = frame.getRedundancyInformation();

			IParameterSet red = params.get(params.keySet().toArray()[0]).getAllParameters();
			// System.out.println(red.toString());

			for (@SuppressWarnings("rawtypes")
			IParameter p : red) {
				writer.write(p.value() + " ");
			}

		} catch (IOException e) {
			// report
		} finally {
			try {
				writer.close();
			} catch (Exception e) {
			}
		}
	}

	/**
	 * Saves the current tcp frame in the frames folder of the configured data path. Filename is 'F' with an
	 * automatically increasing number according to the frame files already existing in the frames folder.
	 * 
	 * @return Name of the saved frame [without ending]
	 */
	public static String saveFrame() {
		String name = SunriseConnector.getControlGui().getFrameName();
		saveFrame(name, SunriseConnector.getTcpFrame());
		SunriseConnector.getControlGui().setNextFrameName(getNextFrameName());

		return name;
	}

	/**
	 * Loads a frame from the frames folder of the configured data path.
	 * 
	 * @param filename
	 *            name of the frame to be loaded
	 * @return Frame object. Returns null, if the corresponding file does not exist
	 */
	public static Frame loadFrame(String filename) {
		String filePath = DataHandler.DataPath + "/frames/" + filename + ".frm";

		System.out.println("Reading from file '" + filePath + "'");
		Frame result = null;

		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(new FileInputStream(filePath), "utf-8"));

			String[] valuesString = reader.readLine().split(" ");
			double[] valuesDouble = new double[valuesString.length];

			for (int i = 0; i < valuesString.length; i++) {
				valuesDouble[i] = Double.parseDouble(valuesString[i]);
			}

			result = new Frame(valuesDouble[0], valuesDouble[1], valuesDouble[2], Math.toRadians(valuesDouble[3]),
					Math.toRadians(valuesDouble[4]), Math.toRadians(valuesDouble[5]));

			// String[] redundancyString = reader.readLine().split(" ");
			//
			// System.out.println("Redundancies: " + Arrays.toString(redundancyString));
			//
			//
			// Map<String, IRedundancyCollection> params = result.getRedundancyInformation();
			// IRedundancyCollection red = params.get(params.keySet().toArray()[0]);
			//
			// IParameterSet tmp = red.getAllParameters();
			//
			// RobotController.getInstance().getRobot().move(ptp(P1));
			// // params.put(params.keySet().toArray()[0], red);
			//
			// result.setRedundancyInformation(RobotController.getInstance().getRobot(), red);

		} catch (IOException e) {
			System.err.println("could not read data from " + filePath + "/" + filename);
		} finally {
			try {
				reader.close();
			} catch (Exception e) {
			}
		}

		return result;
	}

	public static String getNextJointPosName() {
		String dirPath = DataHandler.DataPath + "/joint_positions/";

		File fileObject = new File(dirPath);
		fileObject.mkdirs();

		ArrayList<String> names = new ArrayList<String>(Arrays.asList(fileObject.list()));

		int number = 0;
		for (int i = names.size() - 1; i >= 0; i--) {
			try {
				int n = Integer.parseInt(names.get(i).replace("J", "").replace(".jnt", "")) + 1;
				if (n > number)
					number = n;
			} catch (Exception e) {
				continue;
			}
		}
		return ("J" + number);
	}

	/**
	 * Saves a joint position in the joints folder of the configured data path.
	 * 
	 * @param name
	 *            Name of the file [without ending]
	 * @param jointPos
	 *            Joint position to be saved
	 */
	public static void saveJointPosition(String name, JointPosition jointPos) {
		String filePath = DataHandler.DataPath + "/joint_positions";

		File fileObject = new File(filePath);
		fileObject.mkdirs();

		System.out.println("Writing to file '" + filePath + "/" + name + ".jnt" + "'");

		BufferedWriter writer = null;
		try {
			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filePath + "/" + name + ".jnt"),
					"utf-8"));
			// write joint position
			writer.write(String.format(Locale.US, "%.3f", jointPos.get(0)));
			for (int i = 1; i < 7; i++) {
				writer.write(String.format(Locale.US, " %.3f", jointPos.get(i)));
			}

		} catch (IOException e) {
			// report
		} finally {
			try {
				writer.close();
			} catch (Exception e) {
			}
		}
	}

	/**
	 * Saves the current joint position in the joints folder of the configured data path. Filename is 'J' with an
	 * automatically increasing number according to the joint position files already existing in the joints folder.
	 * 
	 * @return Name of the saved joint position [without ending]
	 */
	public static String saveJointPosition() {
		String name = SunriseConnector.getControlGui().getJointPosName();
		saveJointPosition(name, SunriseConnector.getRobot().getCurrentJointPosition());
		SunriseConnector.getControlGui().setNextJointPosName(getNextJointPosName());

		return name;
	}

	/**
	 * Loads a joint position from the joints folder of the configured data path.
	 * 
	 * @param filename
	 *            name of the joint to be loaded
	 * @return JointPosition object. Returns null, if the corresponding file does not exist
	 */
	public static JointPosition loadJointPos(String filename) {
		String filePath = DataHandler.DataPath + "/joint_positions/" + filename + ".jnt";

		System.out.println("Reading from file '" + filePath + "'");
		JointPosition result = null;

		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(new FileInputStream(filePath), "utf-8"));

			String line = reader.readLine();
			line.replaceAll("  ", " ");
			String[] valuesString = line.split(" ");
			// System.out.println(line);
			// System.out.println("line: " + Arrays.toString(valuesString));
			double[] valuesDouble = new double[valuesString.length];

			for (int i = 0; i < valuesString.length; i++) {
				valuesDouble[i] = Double.parseDouble(valuesString[i]);
			}

			result = new JointPosition(valuesDouble);

		} catch (IOException e) {
			System.err.println("could not read data from " + filePath + "/" + filename);
		} finally {
			try {
				reader.close();
			} catch (Exception e) {
			}
		}

		return result;
	}

}
