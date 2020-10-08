#include "Meca500.h"
#include <cstring>
#include <cstddef>

#include <stdexcept>

#define TIMEOUT_RQ 1000000

#define SET_BIT(prev, bit) (prev | (0x0ff & bit))
#define CLEAR_BIT(prev, bit) (prev & (0x0ff & (~bit)))
#define GET_BIT(mask, value) (mask & value)

namespace sun
{
    std::vector<Meca500 *> Meca500::meca_vector;

    Meca500::Meca500(Master *m, uint16 p, int64 cycletime)
    {
        master = m;
        position = p;
        meca_vector.push_back(this);
        this->cycletime = cycletime;
    }

    Meca500::~Meca500()
    {
        //distruttore
        int i = 0;
        while (this->position != meca_vector[i]->position && i < meca_vector.size())
        {
            i++;
        }
        if (i >= meca_vector.size())
            throw std::runtime_error("");
        meca_vector.erase(meca_vector.begin() + i);
    }

    uint16 Meca500::getPosition()
    {
        return position;
    }

    int64 Meca500::getCycletime()
    {
        return this->cycletime;
    }

    void Meca500::setCycletime(int64 cycletime)
    {
        this->cycletime = cycletime;
    }

    int Meca500::setup(uint16 slave)
    {

        int retval;
        //disattivo i PDO in inviati allo slave
        OBentry TXPDO_number = {0x1c13, 0x00, sizeof(uint8), 0};
        OBentry RXPDO_number = {0x1c12, 0x00, sizeof(uint8), 0};

        printf("\n---DATI DA SCRIVERE---\n");
        retval = ec_SDOwrite(slave, TXPDO_number.index, TXPDO_number.sub_index,
                             FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
        printf("sub_index=%d,value=%x,esito=%d\n\n", TXPDO_number.sub_index, TXPDO_number.value,
               retval);

        retval = ec_SDOwrite(slave, RXPDO_number.index, RXPDO_number.sub_index,
                             FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
        printf("sub_index=%d,value=%x,esito=%d\n\n", RXPDO_number.sub_index, RXPDO_number.value,
               retval);
        //mappo tutti i PDO
        OBentry TXPDO = {0x1c13, 0x01, sizeof(uint16), 0x1a00};
        OBentry RXPDO = {0x1c12, 0x01, sizeof(uint16), 0x1600};

        //posso assegnarli così perchè gli oggetti in cui è descritta la mappatura
        //sono consecutivi
        while (TXPDO.value <= 0x1a08)
        {

            if (TXPDO.value != 0x1a07)
            {
                retval = ec_SDOwrite(slave, TXPDO.index, TXPDO.sub_index, FALSE, TXPDO.size,
                                     &(TXPDO.value), EC_TIMEOUTSAFE);
                printf("sub_index=%d,value=%x,esito=%d\n", TXPDO.sub_index, TXPDO.value, retval);
                TXPDO_number.value++;
                TXPDO.value++;
                TXPDO.sub_index++;
            }

            else
                TXPDO.value++;
        }

        //abilito i PDO

        retval = ec_SDOwrite(slave, TXPDO_number.index, TXPDO_number.sub_index,
                             FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
        printf("esito_numero=%d\n", retval);

        //controllo che siano memorizzati i dati corretti
        printf("\n---DATI LETTI---\n");
        for (int i = 0x1; i <= TXPDO_number.value; i++)
        {
            retval = ec_SDOread(slave, TXPDO.index, i, FALSE, &(TXPDO.size),
                                &(TXPDO.value), EC_TIMEOUTSAFE);
            printf("sub_index=%d,value=%x,esito=%d\n", i, TXPDO.value, retval);
        }

        while (RXPDO.value <= 0x1602)
        {
            retval = ec_SDOwrite(slave, RXPDO.index, RXPDO.sub_index, FALSE, RXPDO.size,
                                 &(RXPDO.value), EC_TIMEOUTSAFE);
            printf("sub_index=%d,value=%x,esito=%d\n", RXPDO.sub_index, RXPDO.value, retval);
            if (retval > 0)
            {
                RXPDO_number.value++;
                RXPDO.value++;
                RXPDO.sub_index++;
            }
        }

        //abilito i PDO
        retval = ec_SDOwrite(slave, RXPDO_number.index, RXPDO_number.sub_index,
                             FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
        printf("esito_numero=%d\n", retval);
        ec_dcsync0(slave, TRUE, cycletime, cycletime / 2);

        return 0;
    }

    int Meca500::setup_static(uint16 position)
    {
        int i = 0;
        while (position != meca_vector[i]->getPosition() && i < meca_vector.size())
        {
            i++;
        }
        if (i >= meca_vector.size())
            throw std::runtime_error("Error setup_Slave\n");
        return meca_vector[i]->setup(position);
    }

    void Meca500::assign_pointer_struct()
    {
        in_MECA500 = (in_MECA500t *)master->getOutput_slave(position);
        out_MECA500 = (out_MECA500t *)master->getInput_slave(position);
    }

    /*
    *                               Request commands
    */

    //PROBLEMA: COME GESTIAMO I SEMAFORI E LE REGIONI CRITICHE??????

    void Meca500::activateRobot()
    {
        if (GET_BIT(0x02, out_MECA500->status_bits == 1))
        {
            std::cout << "Motors already activated.\n";
        }
        else
        {
            int time = 0;
            uint32 activate = 2;
            std::cout << "BIT: " << GET_BIT(0xff, 0x02) << "\n";
            std::cout << sizeof(GET_BIT(0xff, 0x02)) << "\n";
            in_MECA500->robot_control_data = 0x00000002;
            std::cout << "Lettura_struct_activate: " << in_MECA500->robot_control_data << "\n";
            printf("Valore letto_activate_printf: %d\n", GET_BIT(0xff, out_MECA500->status_bits));
            printf("Valore letto_error_printf: %d\n", out_MECA500->error);

            while (GET_BIT(0x02, out_MECA500->status_bits) != 1 && time <= TIMEOUT_RQ)
            {
                usleep(1000);
                time++;
            }
            if (time == TIMEOUT_RQ)
            {
                std::cout << "Timeout exceeded!\n";
                getError();
            }
            else
                std::cout << "Motors activated.\n";
        }
    }

    void Meca500::activateSim()
    {
        //To enable the simulation mode, the robot must be deactivated first.
        int time = 0;
        deactivateRobot();
        in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x05);
        while (GET_BIT(0x04, out_MECA500->status_bits) != 1 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
            getError();
        }
        else
            std::cout << "Simulation mode is enabled.\n";
    }

    void Meca500::deactivateSim()
    {
        int time = 0;
        in_MECA500->robot_control_data = CLEAR_BIT(in_MECA500->robot_control_data, 0x05);
        while (GET_BIT(0x04, out_MECA500->status_bits) != 0 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
            getError();
        }
        else
            std::cout << "Simulation mode is disabled.\n";
    }

    void Meca500::deactivateRobot()
    {
        int time = 0;
        in_MECA500->robot_control_data = CLEAR_BIT(in_MECA500->robot_control_data, 0x06);
        in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x01);
        while (GET_BIT(0x01, out_MECA500->status_bits) != 0 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
            getError();
        }
        else
            std::cout << "Motors are disabled.\n";
    }

    void Meca500::home()
    {
        int time = 0;
        //Verify homing already done
        if (GET_BIT(0x04, out_MECA500->status_bits) == 1)
            std::cout << "Homing already done.\n";
        else
        {
            //Verify if the robot is activated
            if (GET_BIT(0x02, out_MECA500->status_bits == 1))
            {
                //enable homing
                in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x04);
                while (GET_BIT(0x04, out_MECA500->status_bits) != 0 && time <= TIMEOUT_RQ)
                {
                    usleep(1000);
                    time++;
                }
                if (time == TIMEOUT_RQ)
                {
                    std::cout << "Timeout exceeded!\n";
                    getError();
                }
                else
                    std::cout << "Homing done.\n";
            }
            else
            {
                std::cout << "Motors must be activated to do home.\n";
            }
        }
    }

    void Meca500::resetError()
    {
        if (out_MECA500->error = 0)
            std::cout << "There was no error to reset.\n";
        else
        {
            out_MECA500->error = 0;
            std::cout << "The error was reset.\n";
        }
    }

    void Meca500::clearMotion()
    {
        int time = 0;
        in_MECA500->motion_control.motion_control_data = SET_BIT(in_MECA500->motion_control.motion_control_data, 0x40000);
        while (GET_BIT(0x08, out_MECA500->motion_status.motion_bits) != 0 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
        }
        else
            std::cout << "The motion was cleared.\n";
    }

    void Meca500::getConf(int8 *array_c)
    {
        array_c[0] = out_MECA500->configuration.c1;
        array_c[1] = out_MECA500->configuration.c3;
        array_c[2] = out_MECA500->configuration.c5;
    }

    void Meca500::getJoints(float *joint_angles)
    {
        joint_angles[0] = out_MECA500->angular_position.joint_angle_1;
        joint_angles[1] = out_MECA500->angular_position.joint_angle_2;
        joint_angles[2] = out_MECA500->angular_position.joint_angle_3;
        joint_angles[3] = out_MECA500->angular_position.joint_angle_4;
        joint_angles[4] = out_MECA500->angular_position.joint_angle_5;
        joint_angles[5] = out_MECA500->angular_position.joint_angle_6;
    }

    void Meca500::getPose(float *pose)
    {
        pose[0] = out_MECA500->cartesian_position.x;
        pose[1] = out_MECA500->cartesian_position.y;
        pose[2] = out_MECA500->cartesian_position.z;
        pose[3] = out_MECA500->cartesian_position.alpha;
        pose[4] = out_MECA500->cartesian_position.beta;
        pose[5] = out_MECA500->cartesian_position.gamma;
    }

    void Meca500::getStatusRobot(bool &as, bool &hs, bool &sm, bool &es, bool &pm, bool &eob, bool &eom)
    {
        if (GET_BIT(0x02, out_MECA500->status_bits) == 1)
            as = true;
        else
            as = false;

        if (GET_BIT(0x04, out_MECA500->status_bits) == 1)
            hs = true;
        else
            hs = false;

        if (GET_BIT(0x08, out_MECA500->status_bits) == 1)
            sm = true;
        else
            sm = false;

        if (out_MECA500->error == 1)
            es = true;
        else
            es = false;

        if (GET_BIT(0x01, out_MECA500->motion_status.motion_bits) == 1)
            pm = true;
        else
            pm = false;

        if (GET_BIT(0x02, out_MECA500->motion_status.motion_bits) == 1)
            eob = true;
        else
            eob = false;

        if (GET_BIT(0x04, out_MECA500->motion_status.motion_bits) == 1)
            eom = true;
        else
            eom = false;
    }

    void Meca500::pauseMotion()
    {
        int time = 0;
        in_MECA500->motion_control.motion_control_data = SET_BIT(in_MECA500->motion_control.motion_control_data, 0x20000);
        while (GET_BIT(0x01, out_MECA500->motion_status.motion_bits) != 0 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
        }
        else
            std::cout << "Motion paused.\n";
    }

    /*void Meca500::resumeMotion()
    {

    }

    void Meca500::setEOB()
    {
        int time = 0;
        
        while (GET_BIT(0x01, out_MECA500->motion_status.motion_bits) != 0 && time <= TIMEOUT_RQ)
        {
            usleep(1000);
            time++;
        }
        if (time == TIMEOUT_RQ)
        {
            std::cout << "Timeout exceeded!\n";
        }
        else
            std::cout << "Motion paused.\n";
    }

    void Meca500::setEOM()
    {

    }
    
    void Meca500::switchToEthernet()
    {
    }
    */

    /*
    *                               Motion commands
    */

    //ATTENZIONE
    //PER TUTTI I COMANDI CHE SEGUONO VA VERIFICATA SE LA DIMENSIONE DEL VETTORE
    //DI INGRESSO E' QUELLA GIUSTA!!!!
    //DA IMPLEMENTARE

    void Meca500::moveJoints(float *theta)
    {
        //VA FATTO UN CONTROLLO SUI VALORI PASSATI?????
        //AD ESEMPIO,
        in_MECA500->movement.motion_command = 1;
        in_MECA500->movement.variables.varf[0] = theta[0];
        in_MECA500->movement.variables.varf[0] = theta[1];
        in_MECA500->movement.variables.varf[1] = theta[2];
        in_MECA500->movement.variables.varf[2] = theta[3];
        in_MECA500->movement.variables.varf[3] = theta[4];
        in_MECA500->movement.variables.varf[4] = theta[5];
    }

    void Meca500::movePose(float *pose)
    {
        in_MECA500->movement.motion_command = 2;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::moveLin(float *pose)
    {
        in_MECA500->movement.motion_command = 3;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::moveLinRelTRF(float *pose)
    {
        in_MECA500->movement.motion_command = 4;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::moveLinRelWRF(float *pose)
    {
        in_MECA500->movement.motion_command = 5;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::setBlending(float p)
    {
        if (p > 100 || p < 0)
            std::cout << "Error value of percentage! The value must be 0<p<100.\n";
        else
        {

            //0<p<100
            in_MECA500->movement.motion_command = 7;
            in_MECA500->movement.variables.varf[0] = p;
        }
    }

    void Meca500::setJoinVel(float p)
    {
        if (p > 100 || p < 0)
            std::cout << "Error value of percentage! The value must be 0<p<100.\n";
        else
        {
            in_MECA500->movement.motion_command = 8;
            in_MECA500->movement.variables.varf[0] = p;
        }
    }

    void Meca500::setJoinAcc(float p)
    {
        //NEL MANUALE RISULTA P<600 ??????
        if (p > 100 || p < 0)
            std::cout << "Error value of percentage! The value must be 0<p<100.\n";
        else
        {
            in_MECA500->movement.motion_command = 9;
            in_MECA500->movement.variables.varf[0] = p;
        }
    }

    void Meca500::setCartAngVel(float omega)
    {
        if (omega > 300 || omega < 0.001)
            std::cout << "Error value of percentage! The value must be 0.001<p<300.\n";
        else
        {
            in_MECA500->movement.motion_command = 10;
            in_MECA500->movement.variables.varf[0] = omega;
        }
    }

    void Meca500::setCartLinVel(float v)
    {
        if (v > 1000 || v < 0.001)
            std::cout << "Error value of percentage! The value must be 0.001<p<1000.\n";
        else
        {
            in_MECA500->movement.motion_command = 11;
            in_MECA500->movement.variables.varf[0] = v;
        }
    }

    void Meca500::setCartAcc(float p)
    {
        if (p > 100 || p < 0.001)
            std::cout << "Error value of percentage! The value must be 0.001<p<100.\n";
        else
        {
            in_MECA500->movement.motion_command = 12;
            in_MECA500->movement.variables.varf[0] = p;
        }
    }

    void Meca500::setTRF(float *pose)
    {
        in_MECA500->movement.motion_command = 13;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::setWRF(float *pose)
    {
        in_MECA500->movement.motion_command = 14;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
    }

    void Meca500::setConf(float *c)
    {
        in_MECA500->movement.motion_command = 15;
        for (int i = 0; i < 3; i++)
        {
            if (c[i] == 1 || c[i] == -1)
                in_MECA500->movement.variables.varf[i] = c[i];
            else
                i = 4;
        }
        if (i == 4)
            std::cout << "Error values of parameter c! They must be 1 or -1.\n";
    }

    void Meca500::setAutoConf(int e)
    {
        if (e == 1 || e == -1)
        {
            in_MECA500->movement.motion_command = 16;
            in_MECA500->movement.variables.varf[0] = e;
        }
        else
            std::cout << "Error values of parameter e! They must be 1 or -1.\n";
    }

    

    void Meca500::prove()
    {
        uint32 activate = 2;
        *(ec_slave[1].outputs) = activate;
        std::cout << "Lettura_struct_activate: " << in_MECA500->robot_control_data << "\n";
    }

    void Meca500::getError()
    {
        //receive_data(&(out_MECA500.robot_status), offsetof(out_MECA500t, robot_status));
        //int status_error;
        switch (out_MECA500->error)
        {
        case (0):
            std::cout << "Operation succeded\n";
            break;
        case (1013):
            throw std::runtime_error("1013: Activation failed. Try again\n");
            break;
        case (1014):
            throw std::runtime_error("Homing procedure failed.Try again.\n");
            break;
        }
    }

} // namespace sun