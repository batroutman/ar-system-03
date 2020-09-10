import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Point;

import ARPipeline.ARPipeline;
import ARPipeline.Correspondence2D2D;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;
import Jama.SingularValueDecomposition;

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
		// arBootstrapper.start();
		arBootstrapper.tests();
	}

	public void tests() {

		int END_FRAME = 99;

		MockPointData mock = new MockPointData();
		Matrix R1 = mock.getR(0);
		Matrix IC1 = mock.getIC(0);
		Matrix E1 = R1.times(IC1);

		Matrix R2 = mock.getR(END_FRAME);
		Matrix IC2 = mock.getIC(END_FRAME);
		Matrix E2 = R2.times(IC2);

		pl("true E2: ");
		E2.print(15, 4);

		// normalize E2 t
		double norm = E2.getMatrix(0, 2, 3, 3).normF();
		pl("norm:\t\t" + norm);
		E2.set(0, 3, E2.get(0, 3) / norm);
		E2.set(1, 3, E2.get(1, 3) / norm);
		E2.set(2, 3, E2.get(2, 3) / norm);

		pl("E2 normalized: ");
		E2.print(15, 4);

		ArrayList<Correspondence2D2D> corr = new ArrayList<Correspondence2D2D>();
		ArrayList<Point> keypoints1 = mock.getKeypoints(0);
		ArrayList<Point> keypoints2 = mock.getKeypoints(END_FRAME);
		for (int i = 0; i < keypoints1.size(); i++) {
			Correspondence2D2D c = new Correspondence2D2D(keypoints1.get(i).x, keypoints1.get(i).y, keypoints2.get(i).x,
					keypoints2.get(i).y);
			corr.add(c);
		}

		// START FUNCTION
		Matrix A = new Matrix(1, 2);
		Matrix R = E2.getMatrix(0, 2, 0, 2);
		Matrix t = E2.getMatrix(0, 2, 3, 3);

		Matrix x1 = new Matrix(3, 1);
		x1.set(0, 0, keypoints1.get(0).x);
		x1.set(1, 0, keypoints1.get(0).y);
		x1.set(2, 0, 1);

		Matrix x2 = new Matrix(3, 1);
		x2.set(0, 0, keypoints2.get(0).x);
		x2.set(1, 0, keypoints2.get(0).y);
		x2.set(2, 0, 1);

		Matrix x2hat = new Matrix(3, 1);
		x2hat.set(0, 0, 1);
		x2hat.set(1, 0, 1);
		x2hat.set(2, 0, -x2.get(0, 0) - x2.get(1, 0));

		pl("x2: ");
		x2.print(15, 4);

		pl("x2hat: ");
		x2hat.print(15, 4);

		pl("x2hatT * x2: ");
		x2hat.transpose().times(x2).print(15, 4);

		A.set(0, 0, x2hat.transpose().times(R).times(x1).get(0, 0));
		A.set(0, 1, x2hat.transpose().times(t).get(0, 0));

		SingularValueDecomposition svd = A.svd();
		pl("A V:");
		svd.getV().print(15, 4);

		pl("A*V[1] (should be 0): ");
		A.times(svd.getV().getMatrix(0, 1, 1, 1)).print(15, 4);

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}