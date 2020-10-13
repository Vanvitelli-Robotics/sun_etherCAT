#include "Controller.h"


namespace sun
{
    Controller::Controller(Meca500 *meca500, float gain)
    {
        this->gain = gain;
        this->meca500 = meca500;
    }

    Controller::~Controller()
    {
    }

    void Controller::startThread(float *y_d, int dim)
    {
        this->y_d=y_d;
        this->iteration=dim;
        thread_controller = std::thread(&Controller::position_loop_control, this);
    }

    void Controller::position_loop_control()
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 30, 0);

        std::cout << "Start real time controller thread\n";

        float error;
        float joint_position_measured;
        float omega[6] = {0, 0, 0, 0, 0, 0};
        float joint[6] = {0, 0, 0, 0, 0, 0};
    
        for (int i = 0; i < iteration; i++)
        {
            meca500->getJoints(joint);
            joint_position_measured = joint[6];
            

            //error = y_d[i] - ?;
            omega[6] = error * gain;
            meca500->moveJointsVel(omega);
        }
    }

    void Controller::waitLoop()
    {
        thread_controller.join();
    }
} // namespace sun