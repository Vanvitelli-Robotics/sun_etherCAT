#include <iostream>
#include "Master.h"
#include "Meca500.h"
#include "ATINano43.h"
#include <functional>
#include <stdexcept>
#include <chrono>

using namespace sun;
typedef int (Meca500::*meca_fun_ptr)(uint16);
using namespace std;
using namespace EtherCAT;
int main(int argc, char *argv[])
{
    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    if (argc > 1)
    {
        char *ifname = argv[1];
        uint16 state_check;

        try
        {
            Master master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
            ATINano43 forceSensor(1, &master, 1000000);
            Meca500 meca500(2, &master);

            master.setupSlave(meca500.getPosition(), Meca500::setup_static);

            master.configDC();
            master.configMap();
            //printf("in_MECA500_master_main_printf: %x\n",ec_slave[1].inputs );
            //std::cout << "in_MECA500_master_main: " <<ec_slave[1].inputs << "\n";
            meca500.assign_pointer_struct();
            forceSensor.assign_pointer_struct();
            try
            {
                master.movetoState(forceSensor.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.createThread(1000000);

                try
                {
                    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    master.movetoState(forceSensor.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    try
                    {
                        //MECA500_CODE
                        bool as;
                        bool hs;
                        bool sm;
                        bool es;
                        bool pm;
                        bool eob;
                        bool eom;
                        float joint_angles[6];
                        int8 c[3];
                        float pose[6];
                        float joints[6] = {0, 0, 0, 0, 0, 0};
                        float joints2[6] = {0, 0, 0, 0, 90, 0};
                        float joints3[6] = {0, 0, 0, 0, -90, 0};

                        sleep(5);

                        meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        printf("\nActivate: %d\n", as);
                        printf("Homed: %d\n", hs);
                        printf("Sim: %d\n", sm);
                        printf("Error: %d\n", es);

                        //meca500.resetError();
                        meca500.activateRobot();

                        sleep(2);

                        meca500.home();

                        sleep(2);

                        meca500.getJoints(joint_angles);

                        for (int i = 0; i < 6; i++)
                        {
                            std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                        }
                        printf("\n\n");

                        sleep(2);

                        meca500.getConf(c);

                        for (int i = 0; i < 3; i++)
                        {
                            printf("c_%d: %hhd\n", i + 1, c[i]);
                        }
                        printf("\n\n");

                        sleep(2);

                        meca500.getPose(pose);

                        for (int i = 0; i < 6; i++)
                        {
                            std::cout << "pose_" << i + 1 << ": " << pose[i] << "\n";
                        }
                        printf("\n\n");

                        sleep(2);
                        meca500.setPoint(1);

                        sleep(2);

                        meca500.moveJoints(joints2);

                        // sleep(2);

                        // meca500.moveJoints(joints);

                        // sleep(2);

                        // meca500.moveJoints(joints3);

                        // sleep(2);
                        // meca500.moveJoints(joints);

                        sleep(2);
                        meca500.setPoint(0);

                        //meca500.resetMotion();

                        sleep(2);

                        meca500.getJoints(joint_angles);

                        for (int i = 0; i < 6; i++)
                        {
                            std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                        }
                        printf("\n\n");

                        sleep(2);

                        meca500.deactivateRobot();

                        sleep(2);

                        meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        printf("\nActivate: %d\n", as);
                        printf("Homed: %d\n", hs);
                        printf("Sim: %d\n", sm);
                        printf("Error: %d\n", es);

                        //ATINANO43_CODE
                        /*forceSensor.start_realtime();
                        double array_forces[3];
                        double array_torques[3];
                        uint32 array_status[3];
                        int y = 0;
                        sleep(1);
                        int prova = 0;
                        while (TRUE)
                        {
                            auto start = std::chrono::system_clock::now();
                            forceSensor.getForces(array_forces);
                            printf("Value: \n");
                            for (int i = 0; i < 3; i++)
                            {
                                printf("F_%d: %f", i, array_forces[i]);
                                printf("\n");
                            }

                            printf("\n");
                            forceSensor.getTorques(array_torques);
                            for (int i = 0; i < 3; i++)
                            {
                                printf("T_%d: %f", i, array_torques[i]);
                                printf("\n");
                            }
                            forceSensor.getStatus(array_status);
                            auto end = std::chrono::system_clock::now();
                            std::cout << "Time diff: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us.\n";
                            printf("\n");
                            printf("\n");
                            y++;
                            sleep(1);
                            //osal_usleep(1000);
                        }
                        forceSensor.stop();*/
                    }
                    catch (const std::runtime_error &e)
                    {
                        cerr << e.what();
                    }

                    usleep(5000000);
                    try
                    {
                        master.close_master();
                        //master.stampa();
                    }
                    catch (const std::runtime_error &e)
                    {
                        std::cout << "Error close_master\n";
                    }
                    master.waitThread();
                }
                catch (const std::runtime_error &e)
                {
                    std::cout << "Error state_transition SAFE_OP-> OP\n";
                }

                //
            }
            catch (const std::runtime_error &e)
            {
                std::cout << "Error state_transition PRE_OP-> SAFE_OP\n";
            }
        }
        catch (const std::runtime_error &e)
        {
            cerr << e.what();
            return -1;
        }
    }
}
