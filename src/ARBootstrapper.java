import java.util.ArrayList;
import java.util.Collections;

import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.ARUtils;
import ARPipeline.CameraIntrinsics;
import ARPipeline.Correspondence2D2D;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {

	}

	public void start() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

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
		Matrix K = Matrix.identity(4, 4);
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

		// Matrix KInv = K.inverse();
		//
		// Matrix worldPoint = new Matrix(4, 1);
		// worldPoint.set(0, 0, -1000);
		// worldPoint.set(1, 0, 1000);
		// worldPoint.set(2, 0, 1000);
		// worldPoint.set(3, 0, 1);
		//
		// Matrix IC = Matrix.identity(4, 4);
		// IC.set(0, 3, 100);
		// IC.set(1, 3, 0);
		// IC.set(2, 3, 0);
		//
		// Matrix goal = new Matrix(3, 1);
		// goal.set(0, 0, worldPoint.get(0, 0) + IC.get(0, 3));
		// goal.set(1, 0, worldPoint.get(1, 0) + IC.get(1, 3));
		// goal.set(2, 0, worldPoint.get(2, 0) + IC.get(2, 3));
		// goal = goal.times(1 / goal.normF());
		// System.out.println("goal: ");
		// goal.print(5, 10);
		//
		// Matrix R = Matrix.identity(4, 4);
		// R.set(1, 1, Math.cos(Math.PI / 4));
		// R.set(2, 2, Math.cos(Math.PI / 4));
		// R.set(1, 2, -Math.sin(Math.PI / 4));
		// R.set(2, 1, Math.sin(Math.PI / 4));
		//
		// Matrix E = R.times(IC);
		// Matrix EInv = E.inverse();

		int END_FRAME = 99;

		// init mock
		MockPointData mock = new MockPointData();

		// get a correspondence for frame 0 and 100 from a list of
		// correspondences
		Correspondence2D2D c = new Correspondence2D2D();
		int corrInd = 1;
		c.setU1(mock.getKeypoints(0).get(corrInd).x);
		c.setV1(mock.getKeypoints(0).get(corrInd).y);
		c.setU2(mock.getKeypoints(100).get(corrInd).x);
		c.setV2(mock.getKeypoints(100).get(corrInd).y);

		// get camera pose for frame 0
		Matrix R1 = mock.getR(0);
		Matrix IC1 = mock.getIC(0);
		Matrix E1 = R1.times(IC1);

		// get camera pose for frame 100
		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);
		IC2.print(10, 5);
		E2.print(10, 5);
		System.out.println("target scale: " + E2.getMatrix(0, 2, 3, 3).normF());

		// normalize translation vector on frame 100 and pretend this is an
		// essential decomposition
		Matrix t2 = E2.getMatrix(0, 2, 3, 3);
		t2 = t2.times(1 / t2.normF());
		E2.set(0, 3, t2.get(0, 0));
		E2.set(1, 3, t2.get(1, 0));
		E2.set(2, 3, t2.get(2, 0));
		E2.print(10, 5);

		ArrayList<Double> scales = new ArrayList<Double>();
		double scaleSum = 0.0;
		int numCorr = 25;
		for (int i = 0; i < numCorr; i++) {
			c.setU1(mock.getKeypoints(0).get(i).x);
			c.setV1(mock.getKeypoints(0).get(i).y);
			c.setU2(mock.getKeypoints(100).get(i).x);
			c.setV2(mock.getKeypoints(100).get(i).y);
			double scale = ARUtils.estimateScale(E1.getMatrix(0, 2, 0, 3), E2.getMatrix(0, 2, 0, 3), c);
			scales.add(scale);
		}
		Collections.sort(scales);
		int q2 = scales.size() / 4;
		int q4 = q2 * 3;
		double avg = 0;
		for (int i = q2; i < q4; i++) {
			avg += scales.get(i);
		}
		avg /= (q4 - q2);

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}