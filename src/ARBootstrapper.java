import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.Point2D;
import ARPipeline.Point3D;
import ARPipeline.Pose;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public void start() {

		// OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, false);
		SingletonPoseBuffer spb = new SingletonPoseBuffer();
		SingletonFrameBuffer sfb = new SingletonFrameBuffer();
		// ARPipeline pipeline = new TestPipeline(ofb, spb, sfb);
		ARPipeline pipeline = new MockPipeline(sfb, spb, sfb);
		OpenGLARDisplay ARDisplay = new OpenGLARDisplay(sfb, spb);

		pipeline.start();
		ARDisplay.displayLoop();

		println("Done.");
	}

	public static void println(Object obj) {
		System.out.println(obj);
	}

	public static void main(String[] args) {
		ARBootstrapper arBootstrapper = new ARBootstrapper();
		arBootstrapper.start();
		// arBootstrapper.tests();

	}

	public void tests() {

		MockPointData mock = new MockPointData();
		ArrayList<Pose> cameras = new ArrayList<Pose>();
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		ArrayList<ArrayList<Point2D>> obsv = new ArrayList<ArrayList<Point2D>>();

		int START_FRAME = 0;
		int END_FRAME = 1;

		// poses
		Matrix R1 = mock.getR(START_FRAME);
		Matrix IC1 = mock.getIC(START_FRAME);
		Matrix E1 = R1.times(IC1);
		Matrix q1 = mock.getQuaternion(START_FRAME);

		java.util.Map<String, Double> map = new HashMap<String, Double>();
		map.put("1,5", 6.8);
		pl(map.get("1" + ",5"));

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}