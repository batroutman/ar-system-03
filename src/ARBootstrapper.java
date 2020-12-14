import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.OfflineFrameBuffer;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import ARPipeline.TUMFrameBuffer;
import ARPipeline.TestPipeline;

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
		// arBootstrapper.start();
		arBootstrapper.tests();

	}

	public void tests() {

		TUMFrameBuffer tum = new TUMFrameBuffer("../datasets/rgbd_dataset_freiburg1_desk/", true, true);

	}

	public static void p(Object s) {
		System.out.print(s);
	}

	public static void pl(Object s) {
		System.out.println(s);
	}
}