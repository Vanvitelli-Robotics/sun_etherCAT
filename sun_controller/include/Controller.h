#include "Meca500.h"
#include "ATINano43.h"
#include <pthread.h>

namespace sun
{
    class Controller
    {
    private:
        Meca500 *meca500;
        float gain;
        float *y_d;
        int iteration;
        std::thread thread_controller;

    public:
        /**
         *Costruttore
        */
        Controller(Meca500 *meca500, float gain);

        /**
         *Distruttore
        */
        ~Controller();

        /**
         * Create thread
        */
        void startThread(float *y_d, int dim);

        /**
         * 
        */ 
       void position_loop_control();

       void waitLoop();
    };
} // namespace sun
