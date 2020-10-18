#include "Controller_force.h"
#include <fstream>
#include <cmath>

namespace sun
{
    Controller_force::Controller_force(Meca500 *meca500, ATINano43 *force_sensor, float gain)
    {
        this->gain = gain;
        this->meca500 = meca500;
        this->force_sensor = force_sensor;
    }

    Controller_force::~Controller_force()
    {
    }

    void Controller_force::startThread()
    {
        thread_Controller_force = std::thread(&Controller_force::position_loop_control, this);
    }

    void Controller_force::force_loop_control()
    {
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 30, 0);

        std::cout << "Start real time Controller_force thread\n";

        float error;
        double array_forces[3];
        double force_d;
        double force_measure[3];
        double force_measure_z;
        float pose[6];
        float vel_z;

        int count = 0;
        int i = 0; //iterazioni
        int sample = 0;

        //test variables
        float data_forces_z[50000];
        float data_error[50000];
        float data_vel_z[50000];
        float data_measured_vel[50000];
        int64 time_x[50000];
        int64 istant_time;
        int64 start_time;

        start_time = ec_DCtime;
        while (count < 1)
        {
            istant_time = ec_DCtime - start_time;
            force_sensor.getForces(force_measure);
            force_measure_z = force_measure[2];

            error = force_d - force_measure;
            vel_z = gain * error;
            pose[2] = vel_z;
            meca500->moveLinVelTRF(pose); //oppure moveLinVelWRF a seconda del sistema di riferimento

            if (i < 50000)
            {
                time_x[i] = istant_time;
                data_forces_z[i] = force_measure_z;
                data_error[i] = error;
                data_vel_z[i] = vel_z;
                //data_measured_vel[i] = joint_velocities[5];
            }

            i++;
            osal_usleep(1000);
        }

        print_file("Data_forces_z.txt", 50000, data_forces_z);
        print_file("Time_x.txt", 50000, time_x);
        print_file("Data_error.txt", 50000, data_error);
        print_file("Data_velocities_z.txt", 50000, data_vel_z);

        /*
        std::ofstream oFile_p("Data_forces_z.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_p.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_p << data_forces_z[y] << "\t";
                oFile_p << time_x[y] - time_x[0] << "\n";
            }
            oFile_p.close();
        }

        std::ofstream oFile_e("Data_error.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_e.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_e << data_error[y] << "\t";
                oFile_e << time_x[y] - time_x[0] << "\n";
            }
            oFile_e.close();
        }

        std::ofstream oFile_v("Data_velocities.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile_v.is_open())
        {
            if (i > 50000)
                i = 50000;
            for (int y = 0; y < i; y++)
            {
                oFile_v << data_vel[y] << "\t";
            }
            oFile_v.close();
        }*/
    }

    void Controller_force::waitLoop()
    {
        thread_Controller_force.join();
    }

    void Controller_force::print_file(String name, int dim, float array)
    {
        std::ofstream oFile_e(name, std::ios_base::out | std::ios_base::trunc);
        if (oFile_e.is_open())
        {
            for (int y = 0; y < dim; y++)
            {
                oFile_e << array[y] << "\t";
            }
            oFile_e.close();
        }

    } // namespace sun