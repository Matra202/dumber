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
#define PRIORITY_TMOVE 21
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 20
#define PRIORITY_TBATTERY 31
#define PRIORITY_TRELOADWD 32
#define PRIORITY_TRESETSYSTEM 19

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

Camera camera;
bool arenaSearch;
bool arenaValid;
bool startPosition;
int idMachine = 6;  
// on se donne l'ID du rasberry en question en dur pour check si l'ID robot trouvé est celui du nôtre


/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;
    compteur = 0;
    m_withWatchdog = false;
    camera = Camera();
    arenaSearch = false;
    arenaValid = false;
    
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
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
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
    if (err = rt_sem_create(&sem_reloadWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_resetSystem, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_camera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena_confirmation, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
  if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_resetSystem, "th_resetSystem", 0, PRIORITY_TRESETSYSTEM, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
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
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::ReadBattery, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_resetSystem, (void(*)(void*)) & Tasks::ResetSystem, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::GrabCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
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
    /* The task server starts here sem_serverOk                                                       */
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
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        if (msg->CompareID(MESSAGE_CAM_IMAGE)) {
            
        }
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            //cout << msgRcv->ToString() << endl << flush;
            rt_sem_v(&sem_resetSystem);
            rt_sem_p(&sem_serverOk, TM_INFINITE);
            
            //delete(msgRcv);
            //exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            m_withWatchdog = false;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            m_withWatchdog = true;
            rt_sem_v(&sem_startRobot);
        /*} else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)) {
            cout << "\n\n\n\n\n\n\nje suis la je suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la icije suis la iciici" << endl;
            Close_communication_robot();
            Stop();
        */
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            cout << "Opening camera" << endl << flush;
            
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            if (camera.Open()) {
                cout << "Camera opened" << endl << flush;
            } else {
                cout << "Failed to open camera" << endl << flush;
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
            }
            rt_mutex_release(&mutex_camera);
            
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            cout << "Closing camera" << endl << flush;
            
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera.Close();
            rt_mutex_release(&mutex_camera);
            
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
            cout << "Camera closed" << endl << flush;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            cout << "Closing camera" << endl << flush;
            
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            arenaSearch = true;
            rt_mutex_release(&mutex_camera);
            
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
            cout << "Camera closed" << endl << flush;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            if (arenaSearch) arenaValid = true;
            rt_sem_broadcast(&sem_arena_confirmation);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            arenaValid = false;
            rt_sem_broadcast(&sem_arena_confirmation);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            startPosition = true;
            rt_mutex_release(&mutex_camera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            startPosition = false;
            rt_mutex_release(&mutex_camera);
        }
        delete(msgRcv); // must be deleted manually, no consumer
    }
}
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
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        
        if(m_withWatchdog){
            //msgSend = robot.Write(robot.StartWithoutWD());
            cout << "Start robot with watchdog (";
            msgSend = MyWrite(robot.StartWithWD());
        }
        else{
            //msgSend = robot.Write(robot.StartWithoutWD());
            cout << "Start robot without watchdog (";
            msgSend = MyWrite(robot.StartWithoutWD());
            
        }
        
        rt_mutex_release(&mutex_robot);
        
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            if(m_withWatchdog){
                rt_sem_v(&sem_reloadWD);
            }
        }
    }
}


//faut creer une method pour periodiquement envoyer robot.reloadWD

void Tasks::ReloadWD(){
    Message * msgSend;
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
 
    rt_sem_p(&sem_reloadWD, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, RELOADWD_PERIOD);

    while(1){

        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            //cout << "\n\n 55555555555555555555555555555555555555" << endl;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = MyWrite(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
            cout << "\nWatchdog answer: " << msgSend->ToString() << endl << flush;
        }

    }
   
}

Message *Tasks::MyWrite(Message* msg)
{
    Message *msgSend = nullptr;
    
    //rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    cout << "\ntosend=" << msg->ToString() << endl << flush;
    msgSend = robot.Write(msg);
    //rt_mutex_release(&mutex_robot);
    //cout << "\nMessage recu : " << msgSend << endl;
    cout << "answer: " << msgSend->ToString() << endl << flush;
    if( msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) || 
        msgSend->CompareID(MESSAGE_ANSWER_COM_ERROR) ||
        msgSend->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND)){
        compteur++;
        cout << "\ncompteur value: " << compteur << endl;
        if(compteur >= 3){
            Close_communication_robot();
            compteur =0;
        }
    }else {
        compteur =0;
    }
    //cout << "\nMessage successfull, compteur value: " << compteur << endl;

    return msgSend;
}

void Tasks::Close_communication_robot(){
    cout << "Close_communication_robot" << endl; 
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    rt_mutex_release(&mutex_robotStarted);
    robot.Close();
    rt_mutex_release(&mutex_robot);
    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ROBOT_ERROR));
    //Stop();
    //cout << "Stop called" << endl; 

    //Init();
    //cout << "Init called" << endl; 

}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove << "\n";
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            //robot.Write(new Message((MessageID)cpMove));
            MyWrite(new Message ((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread handling status of the robot battery.
 */
void Tasks::ReadBattery(void *arg) {
    int rs;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, BATTERY_PERIOD);
    Message* response = NULL;
            
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery update" << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            //response = robot.Write(robot.GetBattery());
            response = MyWrite(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, response);
        }
    }
}

/**
 * @brief Thread handling periodic image capture.
 */
void Tasks::GrabCamera(void* arg) {
    Img image = camera.Grab();
    int rs;
    bool cameraIsOpen;
    Arena arena;
    std::list<Position> l;
    rt_task_set_periodic(NULL, TM_NOW, CAMERA_PERIOD);
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {
        
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            cameraIsOpen = camera.IsOpen();
            if (cameraIsOpen & !arenaSearch) {
                cout << "Periodic camera grab" << endl << flush;
                Img image2 = camera.Grab();
                image = *image2.Copy();
                if (arenaValid) {
                    image.DrawArena(arena);
                    if (startPosition){
                        l = image.SearchRobot(arena); 
                        if (!l.empty()){
                            bool foundRobot = false;
                            for (std::list<Position>::iterator it = l.begin(); it != l.end(); it++){
                                if(it->robotId == idMachine){
                                    //il s'agit de notre robot 
                                    Position *P = new Position();
                                    P->angle = it->angle;
                                    P->center = it ->center;
                                    P->direction = it->direction;
                                    P->robotId = it->robotId;
                                    image.DrawRobot(*P);
                                    //TODO : SOUCIS ICI AVEC MESSGAE CAM POSITION !!!!!
                                    WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, *P));
                                    foundRobot = true;
                                    break;
                                }
                            }
                            if (!foundRobot){
                                Position *P = new Position();
                                P->center.x = -1.0;
                                P->center.y = -1.0;
                                WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, *P));
                            }
                                
                        }
                        
                    }
                }
                
                if (!image.img.empty()) 
                {
                    cout << "coté normal" << image.ToString() << endl << flush;
                    WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, &image));
                }
            }
            if (cameraIsOpen & arenaSearch) {
                cout << "Grab for arena search" << endl << flush;
                Img image2 = camera.Grab();
                image= *image2.Copy();
                arena = image.SearchArena();
                if (arena.IsEmpty()) {
                    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
                    arenaSearch = false;
                } else {
                    image.DrawArena(arena);
                    
                    if (!image.img.empty()) {
                        cout << "coté arene" << image.ToString() << endl << flush;
                        WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, &image));
                    }
                }
                rt_sem_p(&sem_arena_confirmation, TM_INFINITE);
                arenaSearch = false;
            }
            
            rt_mutex_release(&mutex_camera);
            
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
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

void Tasks::ResetSystem(){
    int err;
    int status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while(1){
        rt_sem_p(&sem_resetSystem, TM_INFINITE);
    
        //rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        //rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        //Stop();
        //rt_mutex_release(&mutex_monitor);
        //rt_mutex_release(&mutex_robot);
        cout << "\n here \n" << flush;
       //rt_task_delete(&th_server);
        //rt_task_delete(&th_sendToMon);
       // rt_task_delete(&th_receiveFromMon);
        /*rt_task_delete(&th_openComRobot);
        rt_task_delete(&th_startRobot);
        rt_task_delete(&th_move);
        rt_task_delete(&th_battery);
        rt_task_delete(&th_reloadWD);*/
        /*if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
            cerr << "Error task create: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }
        if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
            cerr << "Error task start: " << strerror(-err) << endl << flush;
            exit(EXIT_FAILURE);
        }*/
       // Init();
        //Run()
        
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera.Close();
        cout << "\n here2\n" << flush;        
        rt_mutex_release(&mutex_camera);
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        cout << "\n here3\n" << flush;
        Stop();
        rt_mutex_release(&mutex_robot);
        cout << "\n here4\n" << flush;
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

    
}
