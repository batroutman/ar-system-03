import org.opencv.core.Core;
import org.opencv.core.Mat;

import ARPipeline.ARPipeline;
import ARPipeline.ARUtils;
import ARPipeline.OfflineFrameBuffer;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import ARPipeline.TestPipeline;
import Jama.Matrix;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public void start() {

		OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, false);
		SingletonPoseBuffer spb = new SingletonPoseBuffer();
		SingletonFrameBuffer sfb = new SingletonFrameBuffer();
		ARPipeline pipeline = new TestPipeline(ofb, spb, sfb);
		// ARPipeline pipeline = new MockPipeline(sfb, spb, sfb);
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

		Matrix m = new Matrix(4, 4);
		Mat mat = ARUtils.MatrixToMat(m);
		Mat sub = mat.submat(0, 4, 0, 3);
		Matrix subMatrix = ARUtils.MatToMatrix(sub);
		subMatrix.print(15, 5);

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}