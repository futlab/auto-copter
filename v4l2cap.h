#ifndef V4L2CAP_H
#define V4L2CAP_H
#include <stdint.h>

class V4L2Cap
{
private:
    int fd;
    int width_, height_;
    uint32_t pixfmt_;
public:
    inline int  width() { return  width_; }
    inline int height() { return height_; }
    inline uint32_t pixelFormat() { return pixfmt_; }
    V4L2Cap() : fd(-1) {}
    bool openDevice(const char * deviceName);
    bool initDevice(void (* pi) (const void *p, int size));
    bool start();
    bool stop();
    bool getFrame();
    void loop();
    bool closeDevice();
    bool readFrame();
    bool initMMap();
    void initUserp(unsigned int buffer_size);
    bool uninitDevice();
    void (* processImage) (const void *p, int size);
};


#endif // V4L2CAP_H
