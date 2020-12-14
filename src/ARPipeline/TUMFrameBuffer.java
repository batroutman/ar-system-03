package ARPipeline;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import javax.imageio.ImageIO;

// Offline frame buffer designed to load in images from the TUM dataset
public class TUMFrameBuffer implements FrameBuffer {

	// preloaded frames
	ArrayList<Frame> frames = new ArrayList<Frame>();

	// timestamp and file location for each frame.
	// Ex. "1305031452.791720 rgb/1305031452.791720.png"
	ArrayList<String> frameData = new ArrayList<String>();

	// set to true if frames are to be converted to greyscale on load
	boolean convertToGreyscale = true;

	// set to true to load all frames into the frame buffer before popping any
	boolean preloadFrames = true;

	// set to the directory that contains rgb.txt
	String filepath = "";

	// image dimensions
	int WIDTH = 640;
	int HEIGHT = 480;

	private TUMFrameBuffer() {

	}

	public TUMFrameBuffer(String filepath, boolean convertToGreyscale, boolean preloadFrames) {
		this.filepath = filepath;
		this.convertToGreyscale = convertToGreyscale;
		this.preloadFrames = preloadFrames;

		// sanitize filepath
		if (filepath.charAt(filepath.length() - 1) != '/') {
			this.filepath = filepath + "/";
		}

		this.loadFramePaths();

		if (preloadFrames) {
			this.loadAllFrames();
		}

	}

	public void pushFrame(Frame frame) {
		this.frames.add(frame);
	}

	public Frame getCurrentFrame() {
		if (this.preloadFrames) {
			return this.getPreloadedFrame();
		} else {
			return this.loadNextFrame();
		}
	}

	private Frame getPreloadedFrame() {
		if (this.frames.size() > 0) {
			this.frameData.remove(0);
			return this.frames.remove(0);
		} else {
			return null;
		}
	}

	// load a single frame based on the current frame data
	private Frame loadNextFrame() {
		if (this.frameData.size() > 0) {
			String data = this.frameData.remove(0);

			String splits[] = data.split(" ");
			long timestamp = (long) (Double.parseDouble(splits[0]) * 1000000);
			timestamp *= 1000;
			Frame frame = this.loadFrame(this.filepath + splits[1]);
			frame.setTime(timestamp);

			return frame;

		} else {
			return null;
		}
	}

	public void loadFramePaths() {
		BufferedReader reader;
		try {
			reader = new BufferedReader(new FileReader(this.filepath + "rgb.txt"));
			String line = "";
			while (line != null) {
				line = line.trim();
				if (line.length() != 0 && line.charAt(0) != '#') {

					this.frameData.add(line);

				}
				line = reader.readLine();
			}
		} catch (Exception e) {
			System.out.println("Problem loading file paths in TUMFrameBuffer::loadFilePaths()");
			e.printStackTrace();
		}
	}

	public void loadAllFrames() {

		for (int i = 0; i < this.frameData.size() && i < 1; i++) {

			// get the timestamp
			String splits[] = this.frameData.get(i).split(" ");
			long timestamp = (long) (Double.parseDouble(splits[0]) * 1000000);
			timestamp *= 1000;

			Frame frame = this.loadFrame(this.filepath + splits[1]);

			frame.setTime(timestamp);

			this.frames.add(frame);

		}

	}

	public Frame loadFrame(String fullPath) {

		// get the pixel data
		BufferedImage img = null;
		try {
			img = ImageIO.read(new File(fullPath));
		} catch (IOException e) {
			System.out.println("Problem loading image in TUMFrameBuffer::loadAllFrames()");
			e.printStackTrace();
		}

		byte[] r = new byte[this.WIDTH * this.HEIGHT];
		byte[] g = new byte[this.WIDTH * this.HEIGHT];
		byte[] b = new byte[this.WIDTH * this.HEIGHT];
		byte[] grey = new byte[this.WIDTH * this.HEIGHT];

		for (int y = 0; y < this.HEIGHT; y++) {
			for (int x = 0; x < this.WIDTH; x++) {
				byte[] pixelBytes = ByteBuffer.allocate(4).putInt(img.getRGB(x, y)).array();
				int index = this.WIDTH * y + x;
				r[index] = pixelBytes[1];
				g[index] = pixelBytes[2];
				b[index] = pixelBytes[3];
				grey[index] = (byte) ((pixelBytes[1] + pixelBytes[2] + pixelBytes[3]) / 3);
			}
		}

		Frame frame = new Frame();

		frame.setWidth(this.WIDTH);
		frame.setHeight(this.HEIGHT);
		frame.setR(r);
		frame.setG(g);
		frame.setB(b);
		frame.setGrey(grey);

		return frame;

	}

}
