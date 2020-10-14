#include "Controller.h"
#include <fstream>

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

    void Controller::startThread(float y_d)
    {
        this->y_d = y_d;
        thread_controller = std::thread(&Controller::position_loop_control, this);
    }

    void Controller::position_loop_control()
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 30, 0);

        std::cout << "Start real time controller thread\n";

        int count = 0;
        float error;
        float joint_position_measured;
        float omega[6] = {0, 0, 0, 0, 0, 0};
        float joint[6] = {0, 0, 0, 0, 0, 0};
        float data_position_joint[500000];
        float data_error[500000];
        int i = 0; //iterazioni
        int sample = 0;
        int64 time_x[500000];

        while (count < 20)
        {
            //measured position
            meca500->getJoints(joint);
            joint_position_measured = joint[5];

            error = y_d - joint_position_measured;

            if (i < 500000)
            {
                time_x[i] = ec_DCtime;
                data_position_joint[i] = joint_position_measured;
                data_error[i] = error;
            }

            omega[5] = error * gain;
            meca500->moveJointsVel(omega);

            if (error < 0.01 && error > -0.01)
                count++;
            else
                count = 0;
            i++;
            osal_usleep(1000);
        }

        std::ofstream oFile_p("Data_position_Joint6.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_p.is_open())
        {
            if (i > 500000)
                i = 500000;
            oFile_p << "Y_d: " << y_d << "\n";
            for (int y = 0; y < i; y++)
            {
                oFile_p << data_position_joint[y] << "\t";
                oFile_p << time_x[y] << "\n";
            }
            oFile_p.close();
        }

        std::ofstream oFile_e("Data_error.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_e.is_open())
        {
            for (int y = 0; y < i; y++)
            {
                oFile_e << data_error[y] << "\t";
                oFile_e << time_x[y] - time_x[0] << "\n";
            }
            oFile_e.close();
        }
    }

    void Controller::waitLoop()
    {
        thread_controller.join();
    }
} // namespace sun