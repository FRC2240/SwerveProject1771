#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class LimeLight
{
    /******************************************************************/
    /*                       Private Variables                        */
    /******************************************************************/
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

public:
    /******************************************************************/
    /*                        Public Constants                        */
    /******************************************************************/
    enum class LED_Mode
    {
        Keep_Current = 0,
        Force_Off,
        Blink,
        Force_On
    };
    enum class Camera_Mode
    {
        Vision_Processor = 0,
        Driver_Camera
    };
    enum class Stream_Mode
    {
        Standard = 0,
        PiP_Main,
        PiP_Secondary
    };
    enum class Snapshot_Mode
    {
        OFF = 0,
        ON
    };

    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    [[nodiscard]] double getX() const;    // [-27.0, 27.0] For LimeLight-1
    [[nodiscard]] double getY() const;    // [-20.5, 20.5] For LimeLight-1
    [[nodiscard]] double getArea() const; // % of image
    [[nodiscard]] double getSkew() const; // [-90, 0]
    [[nodiscard]] bool hasTarget() const;
    [[nodiscard]] int getShortSideLength() const;      // [0, 320]
    [[nodiscard]] int getLongSideLength() const;       // [0, 320]
    [[nodiscard]] int getHorizontalSideLength() const; // [0, 320]
    [[nodiscard]] int getVerticalSideLength() const;   // [0, 320]
    // Camera Translation not implimented because of unclear documentation
    // Corners not implimented because slightly difficult and we arent using it here
    // Advanced Usage section not implimented becuase im too bored to do so

    [[nodiscard]] int getPipe() const;
    [[nodiscard]] double getLatency_ms() const;

    void setLEDMode(LED_Mode const &mode);
    void setCameraMode(Camera_Mode const &mode);
    void setPipeline(unsigned const &pipe);
    void setStream(Stream_Mode const &mode);
    void setSnapshot(Snapshot_Mode const &mode);
};