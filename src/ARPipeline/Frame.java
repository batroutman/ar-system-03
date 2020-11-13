package ARPipeline;

public class Frame {

	// timestamp of the time the frame was generated (nanoseconds)
	private long time;

	// expected width of the image
	private int width;

	// expected height of the image
	private int height;

	// byte array of R channel of image (RGB)
	private byte[] r;

	// byte array of G channel of image (RGB)
	private byte[] g;

	// byte array of B channel of image (RGB)
	private byte[] b;

	// byte array of Greyscale channel (average of RGB)
	private byte[] grey;

	public Frame() {
		this.time = System.nanoTime();
	}

	public Frame(byte[] grey) {
		this.time = System.nanoTime();
		this.grey = grey;
	}

	public Frame(byte[] grey, int width, int height) {
		this.time = System.nanoTime();
		this.grey = grey;
		this.width = width;
		this.height = height;
	}

	public Frame(byte[] r, byte[] g, byte[] b) {
		this.time = System.nanoTime();
		this.r = r;
		this.g = g;
		this.b = b;
		this.grey = new byte[r.length];
		for (int i = 0; i < r.length; i++) {
			this.grey[i] = (byte) ((r[i] + g[i] + b[i]) / 3);
		}
	}

	public Frame(byte[] r, byte[] g, byte[] b, int width, int height) {
		this.time = System.nanoTime();
		this.width = width;
		this.height = height;
		this.r = r;
		this.g = g;
		this.b = b;
		this.grey = new byte[r.length];
		for (int i = 0; i < r.length; i++) {
			this.grey[i] = (byte) ((r[i] + g[i] + b[i]) / 3);
		}
	}

	public Frame(Frame frame) {
		this.time = frame.time;
		this.width = frame.width;
		this.height = frame.height;
		this.r = new byte[frame.r.length];
		this.g = new byte[frame.g.length];
		this.b = new byte[frame.b.length];
		this.grey = new byte[frame.grey.length];
		for (int i = 0; i < frame.r.length; i++) {
			this.r[i] = frame.r[i];
		}
		for (int i = 0; i < frame.g.length; i++) {
			this.g[i] = frame.g[i];
		}
		for (int i = 0; i < frame.b.length; i++) {
			this.b[i] = frame.b[i];
		}
		for (int i = 0; i < frame.grey.length; i++) {
			this.grey[i] = frame.grey[i];
		}
	}

	public int getWidth() {
		return this.width;
	}

	public void setWidth(int width) {
		this.width = width;
	}

	public int getHeight() {
		return this.height;
	}

	public void setHeight(int height) {
		this.height = height;
	}

	public long getTime() {
		return time;
	}

	public void setTime(long time) {
		this.time = time;
	}

	public byte[] getR() {
		return r;
	}

	public void setR(byte[] r) {
		this.r = r;
	}

	public byte[] getG() {
		return g;
	}

	public void setG(byte[] g) {
		this.g = g;
	}

	public byte[] getB() {
		return b;
	}

	public void setB(byte[] b) {
		this.b = b;
	}

	public byte[] getGrey() {
		return grey;
	}

	public void setGrey(byte[] grey) {
		this.grey = grey;
	}
}
