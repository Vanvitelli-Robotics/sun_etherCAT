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
        Master master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
        ATINano43 forceSensor(1, &master, 1000000);
        master.config_ec_sync0(1, TRUE, 1000000, 500000);

        master.configDC();
        master.configMap();

        master.movetoState(forceSensor.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
        master.createThread(1000000);

        master.movetoState(forceSensor.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);

        forceSensor.assign_pointer_struct();
        forceSensor.start_realtime();
        double array_forces[3];
        double array_torques[3];
        uint32 array_status[3];
        int y = 0;
        sleep(1);
        int prova = 0;
        while (y < 5000)
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
            std::cout <<"Time diff: " <<std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()<< "us.\n";
            printf("\n");
            printf("\n");
            y++;
            osal_usleep(1000);
        }
        forceSensor.stop();
        master.close_master();

        master.waitThread();

        /*try
        {
            Master master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
            Meca500 meca500(&master, 1);

            master.setupSlave(meca500.getPosition(), Meca500::setup_static);

            master.configDC();
            master.configMap();
            //printf("in_MECA500_master_main_printf: %x\n",ec_slave[1].inputs );
            //std::cout << "in_MECA500_master_main: " <<ec_slave[1].inputs << "\n";
            meca500.assign_pointer_struct();
            try
            {
                master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.createThread(1000000);

                try
                {
                    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    meca500.activate();
                    //meca500.prove();
                    //master.stampa();
                    //master.close_master();
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
        }*/
    }
}
