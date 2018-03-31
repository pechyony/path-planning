#ifndef _PREDICTION_H_
#define _PREDICTION_H_

#include <chrono>
#include <tuple>

using namespace std;
using namespace std::chrono;

class Road;

// structure that represents Frenet coordinates of the car
struct FrenetState {

    /**
     * Constructor 
     */
    FrenetState() {};

    /**
     * Constructor
     * @param s Coordinate s of the car
     * @param d Coordinate d of the car
     */ 
    FrenetState(float s, float d);

    float s;   // coordinate s of the car
    float d;   // coordinate d of the car
};

// structure that represents the state of the car
struct State {
    /**
     * Constructor 
     */
    State() {};

    /**
     * Constructor
     * @param s Coordinate s of the car
     * @param d Coordinate d of the car
     * @param x Coordinate x of the car
     * @param y Coordinate y of the car
     * @param vx Velocity of the car along x axis
     * @param vy Velocity of the car along y axis
     * @param t Timestamp of update
     */ 
    State(float s, float d, float x, float y, float vx, float vy, milliseconds t);

    FrenetState f;  // Frenet coordinates
    float x;        // x coordinate
    float y;        // y coordinate
    float vx;       // velocity along x axis
    float vy;       // velocity along y axis
    float ax;       // acceleration along x axis
    float ay;       // acceleration along y axis
    milliseconds t; // timestamp
};

// class that represents the car
class Car {
  public:

    /**
     * Constructor
     * @param road Road object where thecar drives
     */ 
    Car(Road& road);

    /**
     * Constructor
     * @param s Coordinate s of the car
     * @param d Coordinate d of the car
     * @param x Coordinate x of the car
     * @param y Coordinate y of the car
     * @param vx Velocity of the car along x axis
     * @param vy Velocity of the car along y axis
     * @param road Road object where thecar drives
     * @param t Timestamp of update
     */ 
    Car(float s, float d, float x, float y, float vx, float vy, Road& road, milliseconds t);

    /**
     * getLane Return lane of the car
     */
    int getLane();

    /**
     * getSpeed Return speed of the car
     */
    float getSpeed();

    /**
     * getState Return state of the car
     */
    State getState();

    /**
     * update Update state of the car
     * @param s Coordinate s of the car
     * @param d Coordinate d of the car
     * @param x Coordinate x of the car
     * @param y Coordinate y of the car
     * @param vx Velocity of the car along x axis
     * @param vy Velocity of the car along y axis
     * @param t Timestamp of update
     */ 
    void update(float s, float d, float x, float y, float vx, float vy, milliseconds t);

    /**
     * predict Predict location and speed of the car using constant acceleration model
     * @param dt Prediction horizon
     */ 
    tuple<FrenetState,float> predict(float dt);

  private:
    State state;         // current state of the car
    State prev_state;    // previous state of the car
    int lane;            // current lane
    Road& road;          // road where the car drives
};

#endif