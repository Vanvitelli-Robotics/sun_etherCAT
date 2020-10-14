
#include <iostream>
#include "Master.h"
//#include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>

using namespace sun;
typedef int (Meca500::*meca_fun_ptr)(uint16);
using namespace std;

int main(int argc, char *argv[])
{
    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    if (argc > 1)
    {
        char *ifname = argv[1];
        uint16 state_check;

        try
        {
            long time_new_packet[500];
            Master master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
            Meca500 meca500(1, &master);
            Controller controller(&meca500, 0.5);

            master.setupSlave(meca500.getPosition(), Meca500::setup_static);

            master.configDC();
            master.configMap();

            meca500.assign_pointer_struct();

            try
            {
                master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.createThread(1000000);

                try
                {
                    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    try
                    {
                        //MECA500_CODE

                        bool as, hs, sm, es, pm, eob, eom;
                        float joint_angles[6];
                        float joints[6] = {0, 0, 0, 0, 90, 0};
                        float omega[6] = {0, 0, 0, 0, 0, -30};
                        int activateRob, deactivateRob, homeRob;

                        sleep(5);

                        meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        printf("\nActivate: %d\n", as);
                        printf("Homed: %d\n", hs);
                        printf("Sim: %d\n", sm);
                        printf("Error: %d\n", es);

                        meca500.resetError();
                        sleep(2);

                        activateRob = meca500.activateRobot();
                        if (activateRob == 0 || activateRob == 1)
                        {
                            if (activateRob == 1)
                                std::cout << "Motors already activated.\n";
                            else
                                std::cout << "Motors activated.\n";

                            sleep(2);

                            homeRob = meca500.home();
                            if (homeRob == 0 || homeRob == 1)
                            {
                                if (activateRob == 1)
                                    std::cout << "Home already done.\n";
                                else
                                    std::cout << "Home done.\n";
                                sleep(2);

                                meca500.getJoints(joint_angles);

                                for (int i = 0; i < 6; i++)
                                {
                                    std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                                }
                                printf("\n\n");

                                sleep(2);

                                if (meca500.setPoint(1) == 0)
                                {
                                    controller.startThread(180);

                                    controller.waitLoop(); //the user thread waits the end of thread controller.
                                    meca500.setPoint(0);

                                    meca500.getJoints(joint_angles);

                                    for (int i = 0; i < 6; i++)
                                    {
                                        std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                                    }
                                    printf("\n\n");
                                }

                                else
                                    std::cout << "Invalid input!\n";
                            }

                            else
                            {
                                if (homeRob == -1)
                                    std::cout << "Motors must be activated to do home";
                                else
                                    std::cout << "ERROR_Homing.\n";
                            }

                            sleep(2);

                            deactivateRob = meca500.deactivateRobot();
                            if (deactivateRob == 0)
                                std::cout << "Motors deactivated.\n";
                        }
                        else
                        {
                            std::cout << "ERROR_Activate.\n";
                        }

                        sleep(2);

                        // meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        // printf("\nActivate: %d\n", as);
                        // printf("Homed: %d\n", hs);
                        // printf("Sim: %d\n", sm);
                        // printf("Error: %d\n", es);
                    }
                    catch (const std::runtime_error &e)
                    {
                        cerr << e.what();
                    }

                    usleep(5000000);
                    try
                    {
                        master.close_master();
                        master.stampa();
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