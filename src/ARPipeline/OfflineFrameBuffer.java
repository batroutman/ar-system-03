package ARPipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

public class OfflineFrameBuffer implements FrameBuffer {

	protected ArrayList<Frame> frames = new ArrayList<Frame>();
	public boolean YOnly = false;
	protected VideoCapture vc = new VideoCapture();
	protected String filename = "";
	public int targetChunkSize = 1000;
	public long MAX_FRAMES_LOADED = 150;
	public long numFramesLoaded = 0;

	public OfflineFrameBuffer() {

	}

	public OfflineFrameBuffer(boolean YOnly) {
		this.YOnly = YOnly;
	}

	public OfflineFrameBuffer(String filename) {
		this.filename = filename;
		this.loadChunk();
	}

	public OfflineFrameBuffer(String filename, boolean YOnly) {
		this.filename = filename;
		this.YOnly = YOnly;
		this.loadChunk();
	}

	// load a single chunk of the vc, starting where it left off
	public void loadChunk() {
		if (!this.vc.isOpened())
			this.vc.open(filename);
		if (this.vc.isOpened()) {
			Mat mat = new Mat();
			for (int i = 0; this.numFramesLoaded < this.MAX_FRAMES_LOADED && vc.read(mat)
					&& i < this.targetChunkSize; i++) {
				if (i % 50 == 0)
					System.out.println("i: " + i);
				Frame frame = matToFrame(mat, this.YOnly);
				this.frames.add(frame);
				this.numFramesLoaded++;
			}
		}
		System.out.println("Chunk loaded.");
	}

	// convert the given Mat to Frame
	public static Frame matToFrame(Mat mat, boolean greyOnly) {

		int numRows = mat.rows();
		int numCols = mat.cols();
		byte[] grey = new byte[numRows * numCols];
		byte[] r = new byte[numRows * numCols];
		byte[] g = new byte[numRows * numCols];
		byte[] b = new byte[numRows * numCols];

		for (int row = 0; row < numRows; row++) {
			for (int col = 0; col < numCols; col++) {

				double B = mat.get(row, col)[0];
				double G = mat.get(row, col)[1];
				double R = mat.get(row, col)[2];
				double GREY = ((R + G + B) / 3);

				grey[row * numCols + col] = (byte) GREY;
				if (!greyOnly) {
					r[row * numCols + col] = (byte) R;
					g[row * numCols + col] = (byte) G;
					b[row * numCols + col] = (byte) B;
				}

			}
		}

		Frame frame;

		if (greyOnly) {
			frame = new Frame(grey, numCols, numRows);
		} else {
			frame = new Frame(r, g, b, numCols, numRows);
		}

		return frame;
	}

	// setter for filename
	public void setFilename(String filename) {
		this.filename = filename;
	}

	// add frame to buffer
	public void pushFrame(Frame frame) {
		this.frames.add(frame);
	}

	// pop frame from buffer
	public Frame getCurrentFrame() {
		if (this.frames.size() > 0) {
			if (this.frames.size() == 1) {
				this.loadChunk();
			}
			return this.frames.remove(0);

		} else {
			return null;
		}
	}
}
