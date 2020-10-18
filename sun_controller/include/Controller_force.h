#include "Meca500.h"
#include "ATINano43.h"
#include <pthread.h>

namespace sun
{
    class Controller_force
    {
    private:
        Meca500 *meca500;
        ATINano43 *force_sensor;
        float gain;
        std::thread thread_controller;

    public:
        /**
         *Constructor
         @param Meca500 *meca500: the slave to control.
         @param ATINano43 *force_sensor: sensor used to read force values.
         @param float gain: gaint of controller.
        */
        Controller(Meca500 *meca500, ATINano43 *force_sensor, float gain);

        /**
         *Distructor
        */
        ~Controller();

        /**
         * Create thread to start the force control. It calls the force_loop_control method to do it.
        */
        void startThread();

        /**
         * This method realizes the force control of the slave.
        */
        void force_loop_control();

        /**
         *This method waits the termination of the operations of the controller-thread. It must be called after startThread method.
        */
       void waitLoop();

        /**
         * This method must be called to print some data on file.
         * @param String name: name of the file will contain the data.
         * @param int dim: number of data to save into file.
         * @param float array: array contains the data to save.
        */ 
       void print_file(String name, int dim, float array);
       
    };
} // namespace sun
