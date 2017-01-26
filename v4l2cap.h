#ifndef V4L2CAP_H
#define V4L2CAP_H

class V4L2Cap
{
private:
public:
    bool openDevice(const char * deviceName);
    bool initDevice();
    bool start();
    bool getFrame();
    void loop();
};


#endif // V4L2CAP_H
