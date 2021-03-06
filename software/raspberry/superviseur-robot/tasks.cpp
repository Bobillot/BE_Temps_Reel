/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 35
#define PRIORITY_TSENDTOMON 40
#define PRIORITY_TRECEIVEFROMMON 33
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBAT 19
#define PRIORITY_TGESTIMAGE 31
#define PRIORITY_TCALIBARENA 32
#define PRIORITY_TWD 22

//Déclaration des events flags
#define EVENT_INIT 0x0     /* No flags present at init */
#define EVENT_MODE EV_PRIO /* Tasks will wait by priority order */
//start robot
#define EVENT_STARTNOWD 0x1
#define EVENT_STARTWD 0x2
//start watchdog
#define EVENT_SIGNALSTARTWD 0x1 //internal signal to start the watchdog
//Start communication
#define EVENT_COMROBOTSTART 0x1
#define EVENT_COMROBOTSTOP 0x2
#define EVENT_COMROBOTLOST 0x4
//internal gestion_robot_signals
#define EVENT_COMROBOTISSTARTED 0x1    
//Find Arena 
#define EVENT_FINDARENNA 0x1
//arena validity
#define EVENT_ARENAOK 0x1
#define EVENT_ARENANOK 0x2
//camera sending stops
#define EVENT_ENVOIRESUME 0x1
#define EVENT_ENVOISTOP 0x2

//Declaration of event MASK
#define MASK_WAITALL 0xFFFF

//Mode verbose
#define debugTP 1

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;
    camera.SetSize(captureSize::xs);
    /**************************************************************************************/
    /* 	Event creation                                                                   */
    /**************************************************************************************/
    if (err = rt_event_create(&event_comRobot,
            "comRobotStartEvents",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_startRobot,
            "startRobotEvents",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_comRobotStartEvent,
            "ComStatusRobotEvents",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_WD,
            "startWDEvents",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_arenaValid,
            "arenaValidEvents",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_findArena,
            "arena find event",
            EVENT_INIT,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_event_create(&event_envoi,
            "cameraSendingEvents",
            EVENT_ENVOIRESUME,
            EVENT_MODE)) {
        cerr << "Error event create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Events created successfully" << endl << flush;


    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_mutex_create(&mutex_shr_stopRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_shr_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_shr_stopCam, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    //    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
    //        cerr << "Error task create: " << strerror(-err) << endl << flush;
    //        exit(EXIT_FAILURE);
    //    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_batLevelUpdate, "th_batLevelUpdate", 0, PRIORITY_TBAT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_calibration, "th_calibration", 0, PRIORITY_TBAT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    //    if (err = rt_task_create(&th_comRobot, "th_comRobot", 0, PRIORITY_TBAT, 0)) {
    //        cerr << "Error task create: " << strerror(-err) << endl << flush;
    //        exit(EXIT_FAILURE);
    //    }              

    if (err = rt_task_create(&th_WD, "th_WD", 0, PRIORITY_TWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_gestImage, "th_gestImage", 0, PRIORITY_TGESTIMAGE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50000, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::receiveFromMon, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) { //Ce thread remplit le meme role que th_comRobot
        cerr << "Error task start: " << strerror(-err) << endl << flush; // Il doit falloir le supprimer maintenant qu'on en a fait notre version
        exit(EXIT_FAILURE);
    }
    //    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
    //        cerr << "Error task start: " << strerror(-err) << endl << flush;
    //        exit(EXIT_FAILURE);
    //    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_batLevelUpdate, (void(*)(void*)) & Tasks::UpdateBatteryLevel, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_calibration, (void(*)(void*)) & Tasks::Calibration, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    //    if (err = rt_task_start(&th_comRobot, (void(*)(void*)) & Tasks::ThComRobot(), this)) {
    //        cerr << "Error task start: " << strerror(-err) << endl << flush;
    //       exit(EXIT_FAILURE);
    //    }
    if (err = rt_task_start(&th_WD, (void(*)(void*)) & Tasks::ThWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_gestImage, (void(*)(void*)) & Tasks::Gest_Img, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    cout << "Stopping tasks" << endl;
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush; //msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
//void Tasks::ReceiveFromMonTask(void *arg) {
//    Message *msgRcv;
//    
//    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
//    // Synchronization barrier (waiting that all tasks are starting)
//    rt_sem_p(&sem_barrier, TM_INFINITE);
//    
//    /**************************************************************************************/
//    /* The task receiveFromMon starts here                                                */
//    /**************************************************************************************/
//    rt_sem_p(&sem_serverOk, TM_INFINITE);
//    cout << "Received message from monitor activated" << endl << flush;
//
//    while (1) {
//        msgRcv = monitor.Read();
//        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
//
//        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
//            delete(msgRcv);
//            exit(-1);
//        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
//            rt_sem_v(&sem_openComRobot);
//        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
//            rt_sem_v(&sem_startRobot);
//        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
//                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
//                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
//                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
//                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
//
//            rt_mutex_acquire(&mutex_move, TM_INFINITE);
//            move = msgRcv->GetID();
//            rt_mutex_release(&mutex_move);
//        }
//        delete(msgRcv); // mus be deleted manually, no consumer
//    }
//}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    //    while (1) {
    //        rt_sem_p(&sem_openComRobot, TM_INFINITE);
    //        cout << "Open serial com (";
    //        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    //        status = robot.Open();
    //        rt_mutex_release(&mutex_robot);
    //        cout << status;
    //        cout << ")" << endl << flush;
    //
    //        Message * msgSend;
    //        if (status < 0) {
    //            msgSend = new Message(MESSAGE_ANSWER_NACK);
    //        } else {
    //            msgSend = new Message(MESSAGE_ANSWER_ACK);
    //        }
    //        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    //    }

    unsigned int comRobotEventFlag;
    Message * msgSend;
    while (1) {
        rt_event_wait(&event_comRobot, MASK_WAITALL, &comRobotEventFlag, EV_ANY, TM_INFINITE); //:comRobot()?comRobotEventFlag;
        if (comRobotEventFlag == EVENT_COMROBOTSTART) //1 <=> START
        {
            if (debugTP)
                cout << "Com robot start" << endl;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            err = robot.Open();
            rt_mutex_release(&mutex_robot);
            if (err >= 0) {
                msgSend = new Message(MESSAGE_ANSWER_ACK);
                rt_event_signal(&event_comRobotStartEvent, EVENT_COMROBOTISSTARTED); //:comRobotStartEvent!START; 
            } else {
                msgSend = new Message(MESSAGE_ANSWER_NACK);
            }
        } else {
            if (debugTP)
                cout << "stoping com robot" << endl;
            rt_event_clear(&event_comRobotStartEvent, MASK_WAITALL, NULL); //:comRobotStartEvent!STOP;  
            if (comRobotEventFlag == EVENT_COMROBOTLOST) //2<=>LOST 
            {
                if (debugTP)
                    cout << "Com robot lost" << endl;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                err = robot.Close();
                rt_mutex_release(&mutex_robot);
                msgSend = new Message(MESSAGE_ANSWER_ROBOT_TIMEOUT); //ERREUR DANS LA DOC

                err = -1;
                while (err < 0) {
                    //restart connection 
                    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                    err = robot.Open();
                    cout << err << endl;
                    rt_mutex_release(&mutex_robot);
                    cout << "end of loop" << endl;
                }
                if (debugTP)
                    cout << "Com robot reopen" << endl;
                rt_event_signal(&event_comRobot, EVENT_COMROBOTISSTARTED); //mod conception

            } else if (comRobotEventFlag == EVENT_COMROBOTSTOP) //3<=>STOP
            {
                //send stop robot event
                rt_mutex_acquire(&mutex_shr_stopRobot, TM_INFINITE);
                shr_stopRobot = 1;
                rt_mutex_release(&mutex_shr_stopRobot);

                msgSend = new Message(MESSAGE_ROBOT_COM_CLOSE);
                //Mod conception
                cout << "stopped com" << endl;
                rt_event_clear(&event_comRobotStartEvent, MASK_WAITALL, NULL);
            }


        }
        rt_event_clear(&event_comRobot, MASK_WAITALL, NULL);
        //check somewhere here
        WriteInQueue(&q_messageToMon, msgSend);
    }
}

/**
 * @brief Thread starting the communication with the robot. 
 */
//void Tasks::StartRobotTask(void *arg) {
//    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
//    // Synchronization barrier (waiting that all tasks are starting)
//    rt_sem_p(&sem_barrier, TM_INFINITE);
//    /**************************************************************************************/
//    /* The task startRobot starts here                                                    */
//    /**************************************************************************************/
//    while (1) {
//
//        Message * msgSend;
//        rt_sem_p(&sem_startRobot, TM_INFINITE);
//        cout << "Start robot without watchdog (";
//        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
//        msgSend = robot.Write(robot.StartWithoutWD());
//        rt_mutex_release(&mutex_robot);
//        cout << msgSend->GetID();
//        cout << ")" << endl;
//
//        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
//        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
//
//        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
//            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
//            robotStarted = 1;
//            rt_mutex_release(&mutex_robotStarted);
//        }
//    }
//}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    int compteurEchec;
    unsigned int eventReturn;
    int stopRobot;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        //wait for communication started
        compteurEchec = 0;
        eventReturn = 0;
        rt_event_wait(&event_comRobotStartEvent, MASK_WAITALL, &eventReturn, EV_ANY, TM_INFINITE);
        //wait for start robot signal
        rt_event_wait(&event_startRobot, MASK_WAITALL, &eventReturn, EV_ANY, TM_INFINITE);

        cout << "Recvd start robot event" << eventReturn << endl;
        rt_event_clear(&event_startRobot, MASK_WAITALL, NULL);
        Message * msg;
        if (eventReturn == EVENT_STARTWD) {
            rt_event_signal(&event_WD, EVENT_SIGNALSTARTWD);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(ComRobot::StartWithWD());
            rt_mutex_release(&mutex_robot);
        } else if (eventReturn == EVENT_STARTNOWD) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(ComRobot::StartWithoutWD());
            rt_mutex_release(&mutex_robot);
        } else {
            exit(EXIT_FAILURE);
        }


        cout << "Start robot (";
        cout << msg->ToString();
        cout << ")" << endl;

        if (msg->GetID() == MESSAGE_ANSWER_ACK)//robot successfully started
        {
            WriteInQueue(&q_messageToMon, msg);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);


            rt_mutex_acquire(&mutex_shr_stopRobot, TM_INFINITE);
            stopRobot = shr_stopRobot;
            rt_mutex_release(&mutex_shr_stopRobot);

            //Moving loop
            while (compteurEchec < 3 && !stopRobot) {
                rt_task_wait_period(NULL);
                cout << "Periodic movement update";

                //maybe not necesary ?
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                rs = robotStarted;
                rt_mutex_release(&mutex_robotStarted);

                if (rs == 1) {
                    rt_mutex_acquire(&mutex_move, TM_INFINITE);
                    cpMove = move;
                    rt_mutex_release(&mutex_move);

                    cout << " move: " << cpMove;

                    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                    msg = robot.Write(new Message((MessageID) cpMove));
                    rt_mutex_release(&mutex_robot);
                }
                cout << endl << flush;

                //check if message received
                if (msg->GetID() != MESSAGE_ANSWER_ACK && eventReturn == EVENT_STARTWD)
                    compteurEchec++;
                else
                    compteurEchec = 0;

                //actualise stopRobot
                rt_mutex_acquire(&mutex_shr_stopRobot, TM_INFINITE);
                stopRobot = shr_stopRobot;
                rt_mutex_release(&mutex_shr_stopRobot);
            }
            //Stopping robot

            //Stop WD
            rt_event_clear(&event_WD, MASK_WAITALL, NULL);

            //consume event Stop
            rt_mutex_acquire(&mutex_shr_stopRobot, TM_INFINITE);
            shr_stopRobot = 0;
            rt_mutex_release(&mutex_shr_stopRobot);

            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(ComRobot::Reset());
            rt_mutex_release(&mutex_robot);

            //if stopping on connection lost, send message
            if (compteurEchec >= 3) {
                cout << "com lost from robot" << endl;
                rt_event_signal(&event_comRobot, EVENT_COMROBOTLOST);
                //So that it blocks quickly enough
                rt_event_clear(&event_comRobotStartEvent, MASK_WAITALL, NULL);
            }

        } else//error when starting robot
        {
            cout << "Error when starting robot" << endl;
            WriteInQueue(&q_messageToMon, msg);
        }
    }

}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;
    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/*else {
        cout << "@msg :" << msg << endl << flush;
    } /**/
    return msg;
}

/* perso*/

void Tasks::UpdateBatteryLevel() {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    int rs;
    unsigned int retEvent;
    MessageBattery *level;

    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
        rt_event_wait(&event_comRobotStartEvent, EVENT_COMROBOTISSTARTED, &retEvent, EV_ANY, TM_INFINITE);
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            

            //                    msgAnswer = new Message(MESSAGE_ANSWER_ROBOT_TIMEOUT);
            //
            //                    msgAnswer = new Message(MESSAGE_ANSWER_COM_ERROR);
            //TODO check message type to avoid duplicate TO
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            level = (MessageBattery*) robot.Write(ComRobot::GetBattery());
            rt_mutex_release(&mutex_robot);
            
            if(!level->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) && !level->CompareID(MESSAGE_ANSWER_COM_ERROR))
                WriteInQueue(&q_messageToMon, level);
        }
        cout << endl << flush;
    }
}

void Tasks::receiveFromMon() {
    Message * msgRcv;
    bool stopCamera;

    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "started to receive from mon" << endl;
    while (1) {

        //devrait y avoir un mutex, mais bloque (cf code origine)
        msgRcv = monitor.Read();

        switch (msgRcv->GetID()) {
            case (MESSAGE_MONITOR_LOST):
                delete msgRcv;
                cout << "Connection with monitor lost" << endl;
                shr_calculPosition = 0;
                rt_event_signal(&event_comRobot, EVENT_COMROBOTSTOP);
                rt_mutex_acquire(&mutex_shr_stopCam, TM_INFINITE);
                shr_stopCamera = false;
                rt_mutex_release(&mutex_shr_stopCam);
                exit(10);
            case (MESSAGE_ROBOT_COM_OPEN):
                //comRobot!START
                rt_event_signal(&event_comRobot, EVENT_COMROBOTSTART);
                break;
            case (MESSAGE_ROBOT_COM_CLOSE):
                //comRobot!STOP
                rt_event_signal(&event_comRobot, EVENT_COMROBOTSTOP);
                break;
            case (MESSAGE_ROBOT_START_WITH_WD):
                //startRobot!WD
                rt_event_signal(&event_startRobot, EVENT_STARTWD);
                break;
            case (MESSAGE_ROBOT_START_WITHOUT_WD):
                //startRobot!NOWD
                cout << "received start without WD" << endl;
                rt_event_signal(&event_startRobot, EVENT_STARTNOWD);
                break;
            case (MESSAGE_ROBOT_RESET):
                rt_mutex_acquire(&mutex_shr_stopRobot, TM_INFINITE);
                shr_stopRobot = 1;
                rt_mutex_release(&mutex_shr_stopRobot);
                break;
            case (MESSAGE_CAM_OPEN):
                rt_sem_broadcast(&sem_startCamera);
                break;
            case (MESSAGE_CAM_CLOSE):
                rt_mutex_acquire(&mutex_shr_stopCam, TM_INFINITE);
                shr_stopCamera = true;
                rt_mutex_release(&mutex_shr_stopCam);
                break;
            case (MESSAGE_CAM_ASK_ARENA):
                //findArena!
                rt_sem_broadcast(&sem_findArena);
                break;
            case (MESSAGE_CAM_ARENA_CONFIRM):
                //arenaValid!OK
                rt_event_signal(&event_arenaValid, EVENT_ARENAOK);
                break;
            case (MESSAGE_CAM_ARENA_INFIRM):
                //arenaValid!KO
                rt_event_signal(&event_arenaValid, EVENT_ARENANOK);
                break;
            case (MESSAGE_CAM_POSITION_COMPUTE_START):
                //calculPosition!START
                shr_calculPosition = 1;
                break;
            case (MESSAGE_CAM_POSITION_COMPUTE_STOP):
                //calculPosition!STOP
                shr_calculPosition = 0;
                break;
            case (MESSAGE_ROBOT_GO_FORWARD):
            case (MESSAGE_ROBOT_GO_LEFT):
            case (MESSAGE_ROBOT_GO_RIGHT):
            case (MESSAGE_ROBOT_STOP):
            case (MESSAGE_ROBOT_GO_BACKWARD):
                if (debugTP) cout << "received move order" << endl;
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
                break;
        }
        delete msgRcv;
    }
}

void Tasks::Calibration(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    //unsigned int mask_find_arena;
    unsigned int arena_confirmation;
    Img * image;
    Arena * arena;
    Message * msg;
    MessageImg * msgImg;
    while (1) {
        rt_sem_p(&sem_findArena, TM_INFINITE);
        //rt_event_wait(&event_findArena, MASK_WAITALL, &mask_find_arena, EV_ANY, TM_INFINITE); //EV_ANY (OR), EV_ALL (AND)
        cout << "find arena" << endl;

        //clear old arena : 
        rt_mutex_acquire(&mutex_shr_arena, TM_INFINITE);
        arena = shr_arena;
        rt_mutex_release(&mutex_shr_arena);
        if (arena)
            delete arena;

        if (debugTP) cout << "[Thread CALIBRATION] Event flag FindArena received" << endl;
        rt_event_clear(&event_envoi, MASK_WAITALL, NULL); //Stop img sending
        if (debugTP) cout << "[Thread CALIBRATION] Sending ENVOISTOP event flag to camera" << endl;
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        if (!camera.IsOpen()) {
            rt_mutex_release(&mutex_camera);
            continue;
        }
        image = new Img(camera.Grab());
        rt_mutex_release(&mutex_camera);
        if (debugTP) cout << "[Thread CALIBRATION] Capture image" << endl;
        arena = new Arena(image->SearchArena());
        if (debugTP) cout << "[Thread CALIBRATION] Search Arena on the captured image" << endl;

        if (arena->IsEmpty() == true) { //true = no arena found
            msg = new Message(MESSAGE_ANSWER_NACK);
            if (debugTP) cout << "[Thread CALIBRATION] NO ARENA FOUND ! Sending message..." << endl;
            WriteInQueue(&q_messageToMon, msg);
        }
        else {
            rt_event_clear(&event_arenaValid, MASK_WAITALL, NULL);
            if (debugTP) cout << "[Thread CALIBRATION] Arena Found" << endl;
            image->DrawArena(*arena);
            msgImg = new MessageImg(MESSAGE_CAM_IMAGE, image);
            if (debugTP) cout << "[Thread CALIBRATION] Sending arena-image to monitor" << endl;
            WriteInQueue(&q_messageToMon, msgImg);
            if (debugTP) cout << "[Thread CALIBRATION] Waiting for confirmation from monitor..." << endl;
            rt_event_wait(&event_arenaValid, MASK_WAITALL, &arena_confirmation, EV_ANY, TM_INFINITE);
            if (arena_confirmation == EVENT_ARENAOK) {
                if (debugTP) cout << "[Thread CALIBRATION] Arena is OK" << endl;
                rt_mutex_acquire(&mutex_shr_arena, TM_INFINITE);
                shr_arena = arena;
                rt_mutex_release(&mutex_shr_arena);

            }
            else {
                if (debugTP) cout << "[Thread CALIBRATION] Arena is not OK" << endl;
                rt_mutex_acquire(&mutex_shr_arena, TM_INFINITE);
                shr_arena = NULL;
                rt_mutex_release(&mutex_shr_arena);
            }


        }
        rt_event_clear(&event_arenaValid, MASK_WAITALL, NULL);

        if (debugTP) cout << "[Thread CALIBRATION] Sending event flag ENVOIRESUME to camera..." << endl;
        rt_event_signal(&event_envoi, EVENT_ENVOIRESUME);
        
        delete image;
    }
}

void Tasks::Gest_Img() {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    int err;
    SRTIME timeToWait;
    bool sendImages;
    bool stopCamera;
    unsigned int retEvent;
    Message * msg;
    std::list<Position> * pos;



    while (1) {
        rt_sem_p(&sem_startCamera, TM_INFINITE);
        cout << "cam opening" << endl;
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        err = camera.Open();
        rt_mutex_release(&mutex_camera);
        cout << "cam open" << endl;
        if (err != -1) {
            msg = new Message(MESSAGE_ANSWER_ACK);
            WriteInQueue(&q_messageToMon, msg);

            sendImages = true;

            while (sendImages) {
                cout << "Start sending images" << endl;
                rt_mutex_acquire(&mutex_shr_stopCam, TM_INFINITE);
                stopCamera = shr_stopCamera;
                rt_mutex_release(&mutex_shr_stopCam);
                if (stopCamera == 0) {
                    cout << "Waiting event start envoi" << endl;
                    rt_event_wait(&event_envoi, EVENT_ENVOIRESUME, &retEvent, EV_ANY, TM_INFINITE); //peut etre remplacer null_ptr par un unsigned long à définir dans Gest_Img()
                    cout << "Passed event start envoi" << endl;
                    RTIME timeStart = rt_timer_read();

                    int computePosLoc = shr_calculPosition;

                    cout << "Grabbing image" << endl;
                    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                    //need to create new because deleted on message send
                    Img * img = new Img(camera.Grab());
                    rt_mutex_release(&mutex_camera);
                    cout << "Grabbed image" << endl;

                    if (computePosLoc == 1) {

                        rt_mutex_acquire(&mutex_shr_arena, TM_INFINITE);
                        if (shr_arena) //test if arena init
                        {
                            cout << "arena not null" << endl;
                            pos = new std::list<Position>(img->SearchRobot(*shr_arena));
                        } else {
                            pos = new std::list<Position>();
                        }
                        if (pos->empty() || !shr_arena) {
                            pos->back() = *(new Position()); //fuite de mem ?
                            msg = new MessagePosition(MESSAGE_CAM_POSITION, pos->front());
                        } else {
                            msg = new MessagePosition(MESSAGE_CAM_POSITION, pos->front());
                            img->DrawAllRobots(*pos);
                        }
                        rt_mutex_release(&mutex_shr_arena);
                        WriteInQueue(&q_messageToMon, msg);
                    }
                    WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, img));

                    timeToWait = 100000000 - (rt_timer_read() - timeStart);
                    rt_task_sleep(rt_timer_ns2ticks(timeToWait > 0 ? timeToWait : 100000000));

                } else {
                    cout << "Stopping cam" << endl;
                    rt_mutex_acquire(&mutex_shr_stopCam, TM_INFINITE);
                    shr_stopCamera = false;
                    rt_mutex_release(&mutex_shr_stopCam);
                    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                    camera.Close();
                    rt_mutex_release(&mutex_camera);
                    sendImages = false;
                    msg = new Message(MESSAGE_ANSWER_ACK);
                    WriteInQueue(&q_messageToMon, msg);
                }
            }
            cout << "cam out while" << endl;
        } else {
            msg = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msg);
        }
    }
}

void Tasks::ThWD() {
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    unsigned int WDEventFlag;
    while (1) {
        rt_event_wait(&event_WD, MASK_WAITALL, &WDEventFlag, EV_ANY, TM_INFINITE);
        if (WDEventFlag == EVENT_SIGNALSTARTWD) {
            cout << "WD order" << endl;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
            rt_mutex_release(&mutex_robot);
            rt_task_wait_period(NULL);
        }
    }
}


