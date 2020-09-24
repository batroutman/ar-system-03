import java.util.ArrayList;

import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.ARUtils;
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
		// arBootstrapper.start();
		arBootstrapper.tests();
	}

	public void tests() {

		MockPointData mock = new MockPointData();
		ArrayList<Pose> cameras = new ArrayList<Pose>();
		ArrayList<Point3D> point3Ds = new ArrayList<Point3D>();
		ArrayList<ArrayList<Point2D>> observations = new ArrayList<ArrayList<Point2D>>();

		int START_FRAME = 0;
		int END_FRAME = 5;

		// poses
		Matrix R1 = mock.getR(START_FRAME);
		Matrix IC1 = mock.getIC(START_FRAME);
		Matrix E1 = R1.times(IC1);

		Pose pose1 = new Pose();
		pose1.setMatrix(E1);

		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);

		Pose pose2 = new Pose();
		pose2.setMatrix(E2);

		cameras.add(pose1);
		cameras.add(pose2);

		// 3D points and 2D points
		for (int i = 0; i < mock.getWorldCoordinates().size(); i++) {
			Point3D pt3 = new Point3D();
			pt3.setX(mock.getWorldCoordinates().get(i).get(0, 0) + 10);
			pt3.setY(mock.getWorldCoordinates().get(i).get(1, 0) + 10);
			pt3.setZ(mock.getWorldCoordinates().get(i).get(2, 0) + 10);

			point3Ds.add(pt3);

			ArrayList<Point2D> pts2 = new ArrayList<Point2D>();
			Point2D pt21 = new Point2D();
			pt21.setX(mock.getKeypoints(START_FRAME).get(i).x);
			pt21.setY(mock.getKeypoints(START_FRAME).get(i).y);

			Point2D pt22 = new Point2D();
			pt22.setX(mock.getKeypoints(END_FRAME).get(i).x);
			pt22.setY(mock.getKeypoints(END_FRAME).get(i).y);

			pts2.add(pt21);
			pts2.add(pt22);
			observations.add(pts2);
		}

		ARUtils.bundleAdjust(cameras, point3Ds, observations, 5);

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}