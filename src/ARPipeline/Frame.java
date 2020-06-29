package ARPipeline;

public class Frame {

    // timestamp of the time the frame was generated (nanoseconds)
    private long time;
    
    // expected width of the image
    private int width;
    
    // expected height of the image
    private int height;

    // byte array of Y channel of image (YUV)
    private byte [] y;

    // byte array of U channel of image (YUV)
    private byte [] u;

    // byte array of V channel of image (YUV)
    private byte [] v;

    public Frame() {
        this.time = System.nanoTime();
    }

    public Frame(byte [] y) {
        this.time = System.nanoTime();
        this.y = y;
    }

    public Frame(byte [] y, int width, int height) {
        this.time = System.nanoTime();
        this.y = y;
        this.width = width;
        this.height = height;
    }

    
    public Frame(byte [] y, byte [] u, byte [] v) {
        this.time = System.nanoTime();
        this.y = y;
        this.u = u;
        this.v = v;
    }
    
    public Frame(byte [] y, byte [] u, byte [] v, int width, int height) {
        this.time = System.nanoTime();
        this.y = y;
        this.u = u;
        this.v = v;
        this.width = width;
        this.height = height;
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

    public byte[] getY() {
        return y;
    }

    public void setY(byte[] y) {
        this.y = y;
    }

    public byte[] getU() {
        return u;
    }

    public void setU(byte[] u) {
        this.u = u;
    }

    public byte[] getV() {
        return v;
    }

    public void setV(byte[] v) {
        this.v = v;
    }
}
