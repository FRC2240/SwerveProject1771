#include "LimeLight.hpp"
#include <iostream>

// Added comments explaining everything to convert to PhotonVision

// Step 1: Declare and get initial data from camera
double LimeLight::getX() const { return table->GetNumber("tx", 0.0); }
double LimeLight::getY() const { return table->GetNumber("ty", 0.0); }
double LimeLight::getArea() const { return table->GetNumber("ta", 0.0); }
double LimeLight::getSkew() const { return table->GetNumber("ts", 0.0); }
bool   LimeLight::hasTarget() const { return table->GetNumber("tv", 0.0); }
// Step 1b: Get side lengths
int    LimeLight::getShortSideLength() const { return table->GetNumber("tshort", 0.0); }
int    LimeLight::getLongSideLength() const { return table->GetNumber("tlong", 0.0); }
int    LimeLight::getHorizontalSideLength() const { return table->GetNumber("thor", 0.0); }
int    LimeLight::getVerticalSideLength() const { return table->GetNumber("tvert", 0.0); }
// Step 2: Pipeline
int    LimeLight::getPipe() const { return table->GetNumber("getpipe", 0.0); }
double LimeLight::getLatency_ms() const { return table->GetNumber("tl", 0.0) + 11; } // documentation said to add 11ms for image capture
// Step 3: set LEDs, Camera, Stream, Snapshot, Pipeline
void LimeLight::setLEDMode(LimeLight::LED_Mode mode) { table->PutNumber("ledMode", static_cast<int>(mode)); }
void LimeLight::setCameraMode(Camera_Mode mode) { table->PutNumber("camMode", static_cast<int>(mode)); }
void LimeLight::setStream(Stream_Mode mode) { table->PutNumber("stream", static_cast<int>(mode)); }
void LimeLight::setSnapshot(Snapshot_Mode mode) { table->PutNumber("snapshot", static_cast<int>(mode)); }
void LimeLight::setPipeline(unsigned pipe)
{
    if(pipe > 9)
    {
        std::cerr << "invalad pipeline: " << pipe << '\n';
        return;
    }
    table->PutNumber("pipeline", pipe);
}
